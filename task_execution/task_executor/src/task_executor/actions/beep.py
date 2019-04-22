#!/usr/bin/env python
# The beep action in a task plan

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep
from sound_interface import SoundClient

from actionlib_msgs.msg import GoalStatus


class BeepAction(AbstractStep):

    def init(self, name):
        self.name = name
        self._beep_client = SoundClient()
        self._stopped = False

    def run(self, beep, async=False):
        # Check to see if we know about this beep type
        if not isinstance(beep, str) \
                or beep.upper() not in self._beep_client.get_beep_names():
            rospy.logerr("Action {}: FAIL. Unrecognized: {}.".format(self.name, beep))
            raise KeyError(self.name, "Unrecognized", beep)

        self._stopped = False
        beep = beep.upper()
        rospy.loginfo("Action {}: {}".format(self.name, beep))

        # Send the command to play the beep and wait if not async. If async,
        # set as succeeded and exit
        self._beep_client.beep(beep, blocking=False)
        self.notify_action_send_goal(SoundClient.SOUND_PLAY_SERVER, beep)
        if async:
            yield self.set_succeeded()
            raise StopIteration()

        # If we are to block until the client is done running...
        status = GoalStatus.LOST
        while self._beep_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            if self._stopped:
                break

            yield self.set_running()

        # Wait for a result and yield based on how we exited
        status = self._beep_client.get_state()
        result = self._beep_client.get_result(blocking=True)
        self.notify_action_recv_result(SoundClient.SOUND_PLAY_SERVER, status, result)
        if status == GoalStatus.PREEMPTED or self._stopped:
            yield self.set_preempted(
                action=self.name,
                status=status,
                goal=beep,
                result=result
            )
        elif status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        else:
            yield self.set_aborted(
                action=self.name,
                status=status,
                goal=beep,
                result=result
            )

    def stop(self):
        self._stopped = True
        self._beep_client.stop()
        self.notify_action_cancel(SoundClient.SOUND_PLAY_SERVER)
