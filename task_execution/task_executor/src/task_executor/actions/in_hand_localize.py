#!/usr/bin/env python
# Localize a picked object in hand

from __future__ import print_function

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from manipulation_actions.msg import (InHandLocalizeAction as InHandLocalizeMsg,
                                      InHandLocalizeGoal)
from actionlib_msgs.msg import GoalStatus


class InHandLocalizeAction(AbstractStep):
    """
    Localize a ``manipulation_actions/ChallengeObject`` that's in the gripper
    """

    IN_HAND_LOCALIZE_ACTION_SERVER = '/in_hand_localizer/localize'

    def init(self, name):
        self.name = name
        self._localize_client = actionlib.SimpleActionClient(
            InHandLocalizeAction.IN_HAND_LOCALIZE_ACTION_SERVER,
            InHandLocalizeMsg
        )

        rospy.loginfo("Connecting to in_hand_localizer...")
        self._localize_client.wait_for_server()
        rospy.loginfo("...in_hand_localizer connected")

    def run(self, disambiguate_direction=False, verify_schunk_insert=False):
        """
        The run function for this step

        Args:
            disambiguate_direction (bool) : checks point cloud to set the
                x direction as pointing out from the larger side of the object
            verify_schunk_insert (bool) : verifies if the object is grasped in
                a manner that is amenable to insertion in the schunk

        Yields:
            object_transform (geometry_msgs/TransformStamped) :
                the pose of the object in the gripper

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo("Action {}: Localizing object in hand".format(self.name))

        # Create and send the goal
        goal = InHandLocalizeGoal()
        goal.correct_object_direction = disambiguate_direction
        goal.verify_schunk_insert = verify_schunk_insert
        self._localize_client.send_goal(goal)
        self.notify_action_send_goal(
            InHandLocalizeAction.IN_HAND_LOCALIZE_ACTION_SERVER, goal
        )

        # Yield while we're executing
        while self._localize_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Wait for a result and yield based on how we exited
        status = self._localize_client.get_state()
        self._localize_client.wait_for_result()
        result = self._localize_client.get_result()
        self.notify_action_recv_result(
            InHandLocalizeAction.IN_HAND_LOCALIZE_ACTION_SERVER, status, result
        )

        if status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded(object_transform=result.object_transform)
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
        self._localize_client.cancel_goal()
        self.notify_action_cancel(InHandLocalizeAction.IN_HAND_LOCALIZE_ACTION_SERVER)
