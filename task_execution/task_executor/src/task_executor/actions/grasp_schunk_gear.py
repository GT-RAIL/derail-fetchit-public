#!/usr/bin/env python
# Pick up a kit from the table

from __future__ import print_function

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from manipulation_actions.msg import SchunkGraspAction, SchunkGraspGoal, SchunkGraspResult
from actionlib_msgs.msg import GoalStatus


class GraspSchunkGearAction(AbstractStep):
    """
    Grasp the gear inserted in the schunk. This action requires that the pose of the schunk is known,
    preferably through mod:`task_executor.actions.detect_schunk`
    """

    GRASP_SCHUNK_GEAR_ACTION_SERVER = '/schunk_gear_grasper/grasp_schunk_gear'

    def init(self, name):
        self.name = name
        self._grasp_client = actionlib.SimpleActionClient(
            GraspSchunkGearAction.GRASP_SCHUNK_GEAR_ACTION_SERVER,
            SchunkGraspAction
        )

        rospy.loginfo("Connecting to grasp_schunk_gear...")
        self._grasp_client.wait_for_server()
        rospy.loginfo("...grasp_schunk_gear connected")

    def run(self):
        """
        The run function for this step

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo("Action {}: Grasping the gear in the schunk".format(self.name))

        # Create and send the goal
        goal = SchunkGraspGoal()
        self._grasp_client.send_goal(goal)
        self.notify_action_send_goal(
            GraspSchunkGearAction.GRASP_SCHUNK_GEAR_ACTION_SERVER, goal
        )

        # Yield while we're executing
        while self._grasp_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Wait for a result and yield based on how we exited
        status = self._grasp_client.get_state()
        self._grasp_client.wait_for_result()
        result = self._grasp_client.get_result()
        self.notify_action_recv_result(
            GraspSchunkGearAction.GRASP_SCHUNK_GEAR_ACTION_SERVER, status, result
        )

        if status == GoalStatus.SUCCEEDED and result.error_code == SchunkGraspResult.SUCCESS:
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
        self._grasp_client.cancel_goal()
        self.notify_action_cancel(GraspSchunkGearAction.GRASP_SCHUNK_GEAR_ACTION_SERVER)
