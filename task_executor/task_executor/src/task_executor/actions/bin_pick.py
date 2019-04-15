#!/usr/bin/env python
# Pick from the screw bin. Can be configured to be dumb or smart

from __future__ import print_function

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from manipulation_actions.msg import BinPickAction as BinPickMsg, BinPickGoal
from actionlib_msgs.msg import GoalStatus


class BinPickAction(AbstractStep):
    """
    Pick up from the screw bin
    """

    SMART_PICK_ACTION_SERVER = '/cluttered_grasper/smart_bin_pick'
    BLIND_PICK_ACTION_SERVER = '/cluttered_grasper/blind_bin_pick'

    def init(self, name):
        self.name = name
        self._smart_pick_client = actionlib.SimpleActionClient(
            BinPickAction.SMART_PICK_ACTION_SERVER,
            BinPickMsg
        )
        self._blind_pick_client = actionlib.SimpleActionClient(
            BinPickAction.BLIND_PICK_ACTION_SERVER,
            BinPickMsg
        )
        self._chosen_client = None
        self._chosen_name = None

        rospy.loginfo("Connecting to smart_bin_pick...")
        self._smart_pick_client.wait_for_server()
        rospy.loginfo("...smart_bin_pick connected")

        rospy.loginfo("Connecting to blind_bin_pick...")
        self._blind_pick_client.wait_for_server()
        rospy.loginfo("...blind_bin_pick connected")

    def run(self, use_smart=False):
        rospy.loginfo("Action {}: Picking from bin using {}".format(self.name, "SMART" if use_smart else "BLIND"))

        # Create and send the goal
        goal = BinPickGoal()
        self._chosen_name = (
            BinPickAction.SMART_PICK_ACTION_SERVER if use_smart else BinPickAction.BLIND_PICK_ACTION_SERVER
        )
        self._chosen_client = self._smart_pick_client if use_smart else self._blind_pick_client
        self._chosen_client.send_goal(goal)
        self.notify_action_send_goal(self._chosen_name, goal)

        # Yield while we're executing
        while self._chosen_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Wait for a result and yield based on how we exited
        status = self._chosen_client.get_state()
        self._chosen_client.wait_for_result()
        result = self._chosen_client.get_result()
        self.notify_action_recv_result(self._chosen_name, status, result)

        if status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted(
                action=self.name,
                status=status,
                result=result,
                chosen_name=self._chosen_name
            )
        else:
            yield self.set_aborted(
                action=self.name,
                status=status,
                result=result,
                chosen_name=self._chosen_name
            )

        self._chosen_client = None
        self._chosen_name = None

    def stop(self):
        if self._chosen_client is not None:
            self._chosen_client.cancel_goal()
            self.notify_action_cancel(self._chosen_name)
