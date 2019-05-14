#!/usr/bin/env python
# Pick up a kit from the base

from __future__ import print_function

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from manipulation_actions.msg import KitManipAction, KitManipGoal, KitManipResult
from actionlib_msgs.msg import GoalStatus


class PickKitBaseAction(AbstractStep):
    """
    Pickup a kit that has been detected on the table. The action requires that
    the arm is positioned directly above the kit on the base.
    """

    PICK_KIT_BASE_ACTION_SERVER = '/kit_manipulator/pick_kit_base'

    def init(self, name):
        self.name = name
        self._pick_client = actionlib.SimpleActionClient(
            PickKitBaseAction.PICK_KIT_BASE_ACTION_SERVER,
            KitManipAction
        )

        rospy.loginfo("Connecting to pick_kit...")
        self._pick_client.wait_for_server()
        rospy.loginfo("...pick_kit connected")

    def run(self, bin_location):
        """
        The run function for this step

        Args:
            bin_location (str, manipulation_actions/KitManipGoal) : a value
                from the bin pose detection service that indicates where the
                kit that we're trying to locate is placed

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo("Action {}: Picking up the kit from the base".format(self.name))

        # Sanity check the location of the base
        if isinstance(bin_location, str):
            bin_location = getattr(KitManipGoal, bin_location.upper())

        # Create and send the goal
        goal = KitManipGoal(bin_location=bin_location)
        self._pick_client.send_goal(goal)
        self.notify_action_send_goal(
            PickKitBaseAction.PICK_KIT_BASE_ACTION_SERVER, goal
        )

        # Yield while we're executing
        while self._pick_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Wait for a result and yield based on how we exited
        status = self._pick_client.get_state()
        self._pick_client.wait_for_result()
        result = self._pick_client.get_result()
        self.notify_action_recv_result(
            PickKitBaseAction.PICK_KIT_BASE_ACTION_SERVER, status, result
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
        self._pick_client.cancel_goal()
        self.notify_action_cancel(PickKitBaseAction.PICK_KIT_BASE_ACTION_SERVER)
