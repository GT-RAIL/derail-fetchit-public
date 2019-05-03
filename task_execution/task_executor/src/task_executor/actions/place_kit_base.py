#!/usr/bin/env python
# Place a picked up kit on the base of the robot

from __future__ import print_function

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from manipulation_actions.msg import KitManipAction, KitManipGoal, KitManipResult
from actionlib_msgs.msg import GoalStatus


class PlaceKitBaseAction(AbstractStep):
    """
    Place a kit in the gripper on the base. The action requires that the kit
    be present in the gripper, preferrably from running
    :mod:`task_executor.actions.detect_bins`
    """

    PLACE_KIT_BASE_ACTION_SERVER = '/kit_manipulator/place_kit_base'

    def init(self, name):
        self.name = name
        self._place_client = actionlib.SimpleActionClient(
            PlaceKitBaseAction.PLACE_KIT_BASE_ACTION_SERVER,
            KitManipAction
        )

        rospy.loginfo("Connecting to place_kit_base...")
        self._place_client.wait_for_server()
        rospy.loginfo("...place_kit_base connected")

    def run(self):
        """
        The run function for this step

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo("Action {}: Placing the kit on the base".format(self.name))

        # Create and send the goal
        goal = KitManipGoal()
        self._place_client.send_goal(goal)
        self.notify_action_send_goal(
            PlaceKitBaseAction.PLACE_KIT_BASE_ACTION_SERVER, goal
        )

        # Yield while we're executing
        while self._place_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Wait for a result and yield based on how we exited
        status = self._place_client.get_state()
        self._place_client.wait_for_result()
        result = self._place_client.get_result()
        self.notify_action_recv_result(
            PlaceKitBaseAction.PLACE_KIT_BASE_ACTION_SERVER, status, result
        )

        if status == GoalStatus.SUCCEEDED and result.error_code == KitManipResult.SUCCESS:
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
        self._place_client.cancel_goal()
        self.notify_action_cancel(PlaceKitBaseAction.PLACE_KIT_BASE_ACTION_SERVER)
