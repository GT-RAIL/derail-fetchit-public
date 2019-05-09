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

    .. note::

    """

    SCHUNK_INSERT_ACTION_SERVER = "schunk_insert"
    # SCHUNK_WAIT_DURATION = rospy.Duration(122.0)  # Give the thread two seconds more
    # SCHUNK_RETRY_DURATION = rospy.Duration(0.5)   # If we fail to open, then retry every X sec

    def init(self, name):
        self.name = name
        self._schunk_insert_client = actionlib.SimpleActionClient(
            SchunkAction.SCHUNK_INSERT_ACTION_SERVER,
            TwistStamped
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
        rospy.loginfo("Sending command to insert gear into schunk machine chuck")

        # Create and send the goal
        goal = SchunkInsertGoal()
        goal.object_twist_goal.twist.lienar.x = 0.05
        goal.object_twist_goal.twist.linear.y = 0
        goal.object_twist_goal.twist.linear.z = 0
        self._schunk_insert_client.send_goal(goal)
        self.notify_action_send_goal(
            SchunkInsertionAction.SCHUNK_INSERT_ACTION_SERVER, goal
        )

        # Wait for a result and yield based on how we exited
        status = self._schunk_insert_client.get_state()
        self._schunk_insert_client.wait_for_result()
        result = self._schunk_insert_client.get_result()
        self.notify_action_recv_result(
            SchunkInsertionAction.SCHUNK_INSERT_ACTION_SERVER, status, result
        )

        if status == GoalStatus.SUCCEEDED and result.error_code == SchunkInsertResult.SUCCESS:
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
        self.notify_action_cancel(SchunkInsertionAction.SCHUNK_INSERT_ACTION_SERVER)
