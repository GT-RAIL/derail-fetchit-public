#! /usr/bin/env python

# Python
from __future__ import print_function

# ROS
import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from fetchit_challenge.msg import SickCameraAction as SickCameraMsg, SickCameraResult, SickCameraGoal

from task_executor.abstract_step import AbstractStep


class TriggerSickAction(AbstractStep):
    """
    This action triggers the SICK camera. The camera can only be successfully trigered every 5s.
    The action is optional for the task, so we don't care about the result we receive from the
    server when we trigger it.
    """

    SICK_ACTION_SERVER = "sick_camera"

    def init(self, name):
        self.name = name
        self._sick_client = actionlib.SimpleActionClient(
            TriggerSickAction.SICK_ACTION_SERVER,
            SickCameraMsg
        )

        # Initialize the action server and the beliefs action
        rospy.loginfo("Connecting to the sick camera...")
        self._sick_client.wait_for_server()
        rospy.loginfo("...sick camera connected")


    def run(self):
        """
        The run function for this step

        Args:

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo("Action {}: SICK camera triggered!".format(self.name))

        # Creates trigger goal to send to SICK action server
        goal = SickCameraGoal(trigger=SickCameraGoal.TRIG)
        self._sick_client.send_goal(goal)
        self.notify_action_send_goal(TriggerSickAction.SICK_ACTION_SERVER, goal)

        # We don't care about the result from the server, so yield a success
        yield self.set_succeeded()

    def stop(self):
        self._sick_client.cancel_goal()
        self.notify_action_cancel(TriggerSickAction.SICK_ACTION_SERVER)
