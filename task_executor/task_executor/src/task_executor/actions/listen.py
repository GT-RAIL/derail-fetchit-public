#!/usr/bin/env python
# The listen action in a task plan

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from actionlib_msgs.msg import GoalStatus
from hlpr_speech_msgs.srv import SpeechService


class ListenAction(AbstractStep):

    # Currently we use the service to get the speech; perhaps we should use the
    # the topic instead?
    SPEECH_CMD_SERVICE_NAME = "/get_last_speech_cmd"

    def init(self, name):
        self.name = name

        # The speech service
        self._speech_cmd_srv = rospy.ServiceProxy(
            ListenAction.SPEECH_CMD_SERVICE_NAME,
            SpeechService
        )

        # Set a stopped flag
        self._stopped = False

        # Wait for a connection to the speech service
        rospy.loginfo("Connecting to the speech recognition...")
        self._speech_cmd_srv.wait_for_service()
        rospy.loginfo("...speech recognition connected")

    def run(self, expected_cmd=None):
        # Run until we hear a command. If expected_cmd is not None, and the
        # received command is not in the list of expected_cmd, then abort
        rospy.loginfo("Action {}: Listening.{}".format(
            self.name,
            "" if expected_cmd is None else " Expected: {}".format(expected_cmd)
        ))
        self._stopped = False

        received_cmd = None
        while received_cmd is None and not self._stopped:
            try:
                received_cmd = self._speech_cmd_srv(True).speech_cmd
            except rospy.ServiceException as e:
                received_cmd = None

            yield self.set_running()

        # Make only one note of us calling the listen service
        self.notify_service_called(ListenAction.SPEECH_CMD_SERVICE_NAME)

        # Check whether we were stopped. If not, check the expected_cmd and
        # return success or fail accordingly
        if self._stopped:
            yield self.set_preempted(
                action=self.name,
                goal=expected_cmd,
                received_cmd=received_cmd
            )
        elif expected_cmd is not None and received_cmd not in expected_cmd:
            yield self.set_aborted(
                action=self.name,
                goal=expected_cmd,
                received_cmd=received_cmd
            )
        else:  # expected_cmd is None or received_cmd in expected_cmd
            yield self.set_succeeded(cmd=received_cmd)

    def stop(self):
        self._stopped = True
