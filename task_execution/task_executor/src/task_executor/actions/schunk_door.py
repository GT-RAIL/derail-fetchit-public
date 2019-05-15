#!/usr/bin/env python
# Arm approach schunk machine chuck

from __future__ import print_function

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from manipulation_actions.msg import (SchunkDoorAction as SchunkDoorMsg,
                                      SchunkDoorGoal)
from actionlib_msgs.msg import GoalStatus


class SchunkDoorAction(AbstractStep):
    """
    makes the arm open/close the schunk machine door
    """

    SCHUNK_DOOR_ACTION_SERVER = '/schunk_door_node/operate_door'

    def init(self, name):
        self.name = name
        self._door_client = actionlib.SimpleActionClient(
            SchunkDoorAction.SCHUNK_DOOR_ACTION_SERVER,
            SchunkDoorMsg
        )

        rospy.loginfo("Connecting to schunk_door_node...")
        self._door_client.wait_for_server()
        rospy.loginfo("...schunk_door_node connected")

    def run(self, approach_transform, command):
        """
        The run function for this step

        Args:
            approach_transform (geometry_msgs/TransformStamped) : approach pose from schunk detection
            command (str) : "open" or "close" the schunk door

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo("Action {}: Approaching SCHUNK.".format(self.name))

        # Create and send the goal
        goal = ApproachSchunkGoal()
        goal.approach_transform = approach_transform
        self._approach_client.send_goal(goal)
        self.notify_action_send_goal(
            ApproachSchunkAction.APPROACH_SCHUNK_ACTION_SERVER, goal
        )

        # Yield while we're executing
        while self._approach_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Wait for a result and yield based on how we exited
        status = self._approach_client.get_state()
        self._approach_client.wait_for_result()
        result = self._approach_client.get_result()
        self.notify_action_recv_result(
            ApproachSchunkAction.APPROACH_SCHUNK_ACTION_SERVER, status, result
        )

        if status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted(
                action=self.name,
                status=status,
                result=result,
            )
        else:
            yield self.set_aborted(
                action=self.name,
                status=status,
                result=result,
            )

    def stop(self):
        self._approach_client.cancel_goal()
        self.notify_action_cancel(ApproachSchunkAction.APPROACH_SCHUNK_ACTION_SERVER)
