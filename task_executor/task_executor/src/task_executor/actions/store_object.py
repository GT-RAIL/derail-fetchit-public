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
    Store an object in the bin
    """

    STORE_OBJECT_ACTION_SERVER = '/placer/store_object'

    def init(self, name):
        self.name = name
        self._store_client = actionlib.SimpleActionClient(
            StoreObjectAction.STORE_OBJECT_ACTION_SERVER,
            StoreObjectMsg
        )

        rospy.loginfo("Connecting to object_storer...")
        self._store_client.wait_for_server()
        rospy.loginfo("...object_storer connected")

    def run(self, obj):
        rospy.loginfo("Action {}: Storing object {}".format(self.name, obj))

        # Resolve the argument
        if isinstance(obj, str):
            obj = getattr(ChallengeObject, obj.upper())
        else:
            assert isinstance(obj, (int, long,)), "Unknown format for object {}".format(obj)

        # Create and send the goal
        goal = StoreObjectGoal()
        goal.challenge_object.object = obj
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
                goal=obj,
                status=status,
                result=result,
            )
        else:
            yield self.set_aborted(
                action=self.name,
                goal=obj,
                status=status,
                result=result,
            )

    def stop(self):
        self._store_client.cancel_goal()
        self.notify_action_cancel(StoreObjectAction.STORE_OBJECT_ACTION_SERVER)
