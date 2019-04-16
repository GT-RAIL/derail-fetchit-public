#!/usr/bin/env python
# This is soundplay_node v2 where:
#   a) We don't keep pointers to the sinks around (so that we can clean them).
#   b) Conversely, we also (re)check to see if the wav file is available so that
#       we don't cause the alsa system to crash
#   c) We only present the action interface to playing sounds
#   d) We use a full ActionServer; not a simple one
#   e) We don't output diagnostics

from __future__ import print_function, division

import os
import psutil
import threading
import Queue

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from sound_play.msg import (SoundRequest, SoundRequestAction,
                            SoundRequestFeedback, SoundRequestResult)


# The class that encapsulates the action server to play sound requests

class SoundServer(object):
    """
    This is the server that serves sound requests coming in and plays them as
    specified. It does NOT (yet) implement all the functionality available to
    ROS's soundplay_node, or some other soundplay_nodes that I have seen.

    Specifically, this node:
    1. Does not (yet) honour loop requests.
    2. Does not (yet) perform text-to-speech. We assume that has been done by
        the client that is in this same package.
    3. Does not honour the predefined sound files that come with sound_play
    4. Does not allow the specification of sound priorities or overriding. If
        there are multiple sound requests that come in at the same time, they
        are queued and played in order
    """

    DEFAULT_SOUNDPLAY_COMMAND = "play"
    DEFAULT_SOUNDPLAY_ARGS = ["-q"]
    SOUND_SERVER_NAME = "sound_server"

    def __init__(self):
        # Managing the sound requests that come into the action server. This is
        # used to make sure that we can honour stop requests
        self._current_sounds = Queue.Queue()
        self._sound_manager_thread = threading.Thread(target=self._monitor_sounds)

        # Variable to handle stop requests
        self._last_stop_time = rospy.Time(0)

        # The action server
        self._server = actionlib.ActionServer(
            SoundServer.SOUND_SERVER_NAME,
            SoundRequestAction,
            goal_cb=self._on_goal,
            auto_start=False
        )

    def start(self):
        self._server.start()
        self._sound_manager_thread.start()
        rospy.loginfo("{} node is ready to play sound".format(rospy.get_name()))

    def _on_goal(self, gh):
        # First check that the fields are set according to how we expect them
        # to be for the incoming sound requests
        result = gh.get_default_result()
        if not self._verify_goal(gh):
            result.stamp = rospy.Time.now()
            gh.set_rejected(result)
            return

        # Accept the goal, but check to see if it's been preempted
        gh.set_accepted()
        if gh.get_goal_status().status == GoalStatus.PREEMPTING:
            rospy.logdebug("Goal with id {} has been preempted".format(gh.get_goal_id().id))
            result.stamp = rospy.Time.now()
            gh.set_canceled(result)
            return

        # Is this a stop or is this a play?
        if gh.get_goal().sound_request.command == SoundRequest.PLAY_STOP:
            self._stop_playing(gh, result)
        else:
            self._start_playing(gh, result)

    def _stop_playing(self, gh, result):
        self._last_stop_time = rospy.Time.now()
        # Waiting for an ack on the stop request is too hard, so we're just
        # going to mark this as a success and trust that our monitor will have
        # dealt with stopping the sound
        result.stamp = rospy.Time.now()
        gh.set_succeeded(result)

    def _start_playing(self, gh, result):
        # Now open a file pointer to the wav files so that we can be sure the
        # file exists while play has access to it
        try:
            fd = open(gh.get_goal().sound_request.arg, 'rb')
        except Exception as e:
            rospy.logerr("Exception while opening wav file: {}".format(e))
            result.stamp = rospy.Time.now()
            gh.set_aborted(result)

        # We now have access to the file pointer. Now send it on to play (or
        # whatever command we have chosen to play sounds)
        play_command = (
            [SoundServer.DEFAULT_SOUNDPLAY_COMMAND]
            + SoundServer.DEFAULT_SOUNDPLAY_ARGS
            + [gh.get_goal().sound_request.arg]
        )
        play_process = psutil.Popen(play_command)
        self._current_sounds.put((gh, play_process, rospy.Time.now(),))
        gh.publish_feedback(SoundRequestFeedback(playing=True, stamp=rospy.Time.now()))

        # Now wait until we receive a signal that the process has terminated
        while play_process.poll() is None:
            rospy.sleep(0.1)

        # Then check the status we were in. Close the file descriptor first
        fd.close()
        if gh.get_goal_status().status == GoalStatus.PREEMPTING:
            rospy.logwarn("Goal to play {} preempted.".format(gh.get_goal().sound_request.arg))
            result.stamp = rospy.Time.now()
            gh.set_canceled(result)
        elif play_process.poll() != 0:
            rospy.logerr("Goal to play {} aborted.".format(gh.get_goal().sound_request.arg))
            result.stamp = rospy.Time.now()
            gh.set_aborted(result)
        else:  # status == SUCCEEDED and poll() == 0
            rospy.logdebug("Goal to play {} succeeded.".format(gh.get_goal().sound_request.arg))
            result.stamp = rospy.Time.now()
            gh.set_succeeded(result)

    def _monitor_sounds(self):
        # Keep track of all the sounds we're playing. If a cancel comes in, then
        # cancel the sound followed by the goal handle
        while not rospy.is_shutdown():
            rospy.sleep(0.1)

            # Try to get the sounds to process
            try:
                (sound_gh, play_process, time_added,) = self._current_sounds.get(False)
                self._current_sounds.task_done()
            except Queue.Empty as e:
                continue

            # Check to see if we've been canceled. If so, stop playing and
            # let the goal_cb take care of the ROS communication
            if time_added <= self._last_stop_time:
                sound_gh.set_cancel_requested()
                play_process.terminate()
                rospy.logdebug("Play of {} has terminated".format(sound_gh.get_goal().sound_request.arg))
                continue

            # Check to see if we're not done. If so, then add back to the queue
            if play_process.poll() is None:
                self._current_sounds.put((sound_gh, play_process, time_added,))

    def _verify_goal(self, gh):
        goal = gh.get_goal().sound_request

        # Run through the list of settings that would make the goal invalid
        if goal.sound not in [SoundRequest.PLAY_FILE, SoundRequest.ALL]:
            rospy.logerr("This sound server cannot play sound of type: {}".format(goal.sound))
            return False

        if goal.command not in [SoundRequest.PLAY_ONCE, SoundRequest.PLAY_STOP]:
            rospy.logerr("This sound server cannot handle command of type: {}".format(goal.command))
            return False

        if goal.command == SoundRequest.PLAY_STOP and goal.sound != SoundRequest.ALL:
            rospy.logerr("This sound server can only stop all sounds")
            return False

        if goal.command == SoundRequest.PLAY_ONCE and (
            goal.sound != SoundRequest.PLAY_FILE or not os.path.exists(goal.arg)
        ):
            rospy.logerr("The sound request is misconfigured: Cmd - {}, Snd - {}, Arg - {}"
                         .format(goal.command, goal.sound, goal.arg))
            return False

        # All checks passed
        return True
