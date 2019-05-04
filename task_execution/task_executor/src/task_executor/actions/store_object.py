#!/usr/bin/env python
# Localize a picked object in hand

from __future__ import print_function

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from manipulation_actions.msg import (StoreObjectAction as StoreObjectMsg,
                                      StoreObjectGoal, ChallengeObject)
from actionlib_msgs.msg import GoalStatus


class StoreObjectAction(AbstractStep):
    """
    Store a ``manipulation_actions/ChallengeObject`` that is in the gripper in
    the kit, which is on the base. The action requires that the pose of kit is
    known, preferably through :mod:`task_executor.actions.detect_bins` and that
    the pose of the object in the gripper is known, through perhaps
    :mod:`task_executor.actions.in_hand_localize`
    """

    STORE_OBJECT_ACTION_SERVER = '/kit_manipulator/store_object'

    def init(self, name):
        self.name = name
        self._store_client = actionlib.SimpleActionClient(
            StoreObjectAction.STORE_OBJECT_ACTION_SERVER,
            StoreObjectMsg
        )

        rospy.loginfo("Connecting to object_storer...")
        self._store_client.wait_for_server()
        rospy.loginfo("...object_storer connected")

    def run(self, object_key):
        """
        The run function for this step

        Args:
            object_key (str, int) : an identifier of the object to store in the
                kit. If a `str`, then we lookup the corresponding `int`
                identifier for the object from \
                ``manipulation_actions/ChallengeObject``

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo("Action {}: Storing object {}".format(self.name, object_key))

        # Resolve the argument
        if isinstance(object_key, str):
            object_key = getattr(ChallengeObject, object_key.upper())
        else:
            assert isinstance(object_key, (int, long,)), "Unknown format for object {}".format(object_key)

        # Create and send the goal
        goal = StoreObjectGoal()
        goal.challenge_object.object = object_key
        self._store_client.send_goal(goal)
        self.notify_action_send_goal(
            StoreObjectAction.STORE_OBJECT_ACTION_SERVER, goal
        )

        # Yield while we're executing
        while self._store_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Wait for a result and yield based on how we exited
        status = self._store_client.get_state()
        self._store_client.wait_for_result()
        result = self._store_client.get_result()
        self.notify_action_recv_result(
            StoreObjectAction.STORE_OBJECT_ACTION_SERVER, status, result
        )

        if status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted(
                action=self.name,
                goal=object_key,
                status=status,
                result=result,
            )
        else:
            yield self.set_aborted(
                action=self.name,
                goal=object_key,
                status=status,
                result=result,
            )

    def stop(self):
        self._store_client.cancel_goal()
        self.notify_action_cancel(StoreObjectAction.STORE_OBJECT_ACTION_SERVER)
