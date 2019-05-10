#!/usr/bin/env python
# Localize a picked object in hand

from __future__ import print_function

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from manipulation_actions.msg import (ApproachSchunkAction as ApproachSchunkMsg,
                                      ApproachSchunkGoal)
from actionlib_msgs.msg import GoalStatus


class ApproachSchunkAction(AbstractStep):
    """
    Localize a ``manipulation_actions/ChallengeObject`` that's in the gripper
    """

    APPROACH_SCHUNK_ACTION_SERVER = '/approach_schunk_node/approach_schunk'

    def init(self, name):
        self.name = name
        self._approach_client = actionlib.SimpleActionClient(
            ApproachSchunkAction.APPROACH_SCHUNK_ACTION_SERVER,
            ApproachSchunkMsg
        )

        rospy.loginfo("Connecting to approach_schunk_node...")
        self._approach_client.wait_for_server()
        rospy.loginfo("...approach_schunk_node connected")

    def run(self, approach_transform):
        """
        The run function for this step

        Args:
            approach_transform (geometry_msgs/TransformStamped) : the approach transform in the map frame

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
        result = self._localize_client.get_result()
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
