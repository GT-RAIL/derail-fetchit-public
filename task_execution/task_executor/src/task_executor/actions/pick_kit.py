#!/usr/bin/env python
# Pick up a kit from the table

from __future__ import print_function

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from manipulation_actions.msg import KitManipAction, KitManipGoal, KitManipResult
from actionlib_msgs.msg import GoalStatus


class PickKitAction(AbstractStep):
    """
    Pickup a kit that has been detected on the table. The action requires that
    the pose of kit is known, preferably through
    :mod:`task_executor.actions.detect_bins`
    """

    PICK_KIT_ACTION_SERVER = '/kit_manipulator/pick_kit'

    def init(self, name):
        self.name = name
        self._pick_client = actionlib.SimpleActionClient(
            PickKitAction.PICK_KIT_ACTION_SERVER,
            KitManipAction
        )

        rospy.loginfo("Connecting to pick_kit...")
        self._pick_client.wait_for_server()
        rospy.loginfo("...pick_kit connected")

    def run(self):
        """
        The run function for this step

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo("Action {}: Picking up the kit".format(self.name))

        # Create and send the goal
        goal = KitManipGoal()
        self._pick_client.send_goal(goal)
        self.notify_action_send_goal(
            PickKitAction.PICK_KIT_ACTION_SERVER, goal
        )

        # Yield while we're executing
        while self._pick_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Wait for a result and yield based on how we exited
        status = self._pick_client.get_state()
        self._pick_client.wait_for_result()
        result = self._pick_client.get_result()
        self.notify_action_recv_result(
            PickKitAction.PICK_KIT_ACTION_SERVER, status, result
        )

        if status == GoalStatus.SUCCEEDED and result.error_code == KitManipResult.SUCCESS:
            yield self.set_succeeded(kit_grasp_index=result.grasp_index)
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
        self.notify_action_cancel(PickKitAction.PICK_KIT_ACTION_SERVER)
