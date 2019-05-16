#!/usr/bin/env python
# Pick up a kit from the table

from __future__ import print_function

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from manipulation_actions.msg import SchunkRetrieveAction, SchunkRetrieveGoal, SchunkRetrieveResult
from actionlib_msgs.msg import GoalStatus


class RetrieveSchunkGearAction(AbstractStep):
    """
    Retrieve the gear in the schunk. This action requires that
    the gear is being grasped, preferably through
    :mod:`task_executor.actions.grasp_schunk_gear`
    """

    RETRIEVE_SCHUNK_GEAR_ACTION_SERVER = '/schunk_gear_grasper/retrieve_schunk_gear'

    def init(self, name):
        self.name = name
        self._retrieve_client = actionlib.SimpleActionClient(
            RetrieveSchunkGearAction.RETRIEVE_SCHUNK_GEAR_ACTION_SERVER,
            SchunkRetrieveAction
        )

        rospy.loginfo("Connecting to retrieve_schunk_gear...")
        self._retrieve_client.wait_for_server()
        rospy.loginfo("...retrieve_schunk_gear connected")

    def run(self, add_collision_object=True):
        """
        The run function for this step

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo("Action {}: Retrieve the gear in the schunk".format(self.name))

        # Create and send the goal
        goal = SchunkRetrieveGoal()
        goal.add_collision_object = add_collision_object
        self._retrieve_client.send_goal(goal)
        self.notify_action_send_goal(
            RetrieveSchunkGearAction.RETRIEVE_SCHUNK_GEAR_ACTION_SERVER, goal
        )

        # Yield while we're executing
        while self._retrieve_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Wait for a result and yield based on how we exited
        status = self._retrieve_client.get_state()
        self._retrieve_client.wait_for_result()
        result = self._retrieve_client.get_result()
        self.notify_action_recv_result(
            RetrieveSchunkGearAction.RETRIEVE_SCHUNK_GEAR_ACTION_SERVER, status, result
        )

        if status == GoalStatus.SUCCEEDED and result.error_code == SchunkRetrieveResult.SUCCESS:
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
        self._retrieve_client.cancel_goal()
        self.notify_action_cancel(RetrieveSchunkGearAction.RETRIEVE_SCHUNK_GEAR_ACTION_SERVER)
