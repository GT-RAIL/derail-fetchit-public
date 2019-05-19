#!/usr/bin/env python
# Insert peg into the schunk machine

from __future__ import print_function, division

from threading import Thread

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import TwistStamped
from manipulation_actions.msg import SchunkInsertAction, SchunkInsertGoal, SchunkInsertResult

from task_executor.abstract_step import AbstractStep


class SchunkInsertionAction(AbstractStep):
    """
    Inserts the large gear into the schunk machines' chuck once the gear is aligned to the chuck
    """

    SCHUNK_INSERT_ACTION_SERVER = "/schunk_insertion/schunk_insert"

    def init(self, name):
        self.name = name
        self._schunk_insert_client = actionlib.SimpleActionClient(
            SchunkInsertionAction.SCHUNK_INSERT_ACTION_SERVER,
            SchunkInsertAction
        )

        # Initialize the action server and the beliefs action
        rospy.loginfo("Connecting to schunk_insert...")
        self._schunk_insert_client.wait_for_server()
        rospy.loginfo("...schunk_insert connected")

    def run(self):
        """
        The run function for this step

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo("Action {}: inserting gear into SCHUNK machine chuck".format(self.name))

        # Create and send the goal
        goal = SchunkInsertGoal()
        goal.object_twist_goal.twist.linear.x = 0.02
        goal.object_twist_goal.twist.linear.y = 0
        goal.object_twist_goal.twist.linear.z = 0
        self._schunk_insert_client.send_goal(goal)

        # Yield an empty dict while we're executing
        while self._schunk_insert_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Wait for a result and yield based on how we exited
        status = self._schunk_insert_client.get_state()
        self._schunk_insert_client.wait_for_result()
        result = self._schunk_insert_client.get_result()

        if status == GoalStatus.SUCCEEDED and result.success:
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
        self._schunk_insert_client.cancel_goal()
