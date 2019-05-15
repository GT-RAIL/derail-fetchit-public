#!/usr/bin/env python
# Pullback gripper from the schunk machine

from __future__ import print_function, division

from threading import Thread

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from manipulation_actions.msg import SchunkPullbackAction, SchunkPullbackGoal, SchunkPullbackResult

from task_executor.abstract_step import AbstractStep


class SchunkGripperPullbackAction(AbstractStep):
    """
    Releases the large gear and pulls back from the schunk machines' chuck once the chuck is closed

    .. note::

    """

    SCHUNK_PULLBACK_ACTION_SERVER = "/schunk_insertion/schunk_pullback"

    def init(self, name):
        self.name = name
        self._schunk_pullback_client = actionlib.SimpleActionClient(
            SchunkGripperPullbackAction.SCHUNK_PULLBACK_ACTION_SERVER,
            SchunkPullbackAction
        )

        # Initialize the action server and the beliefs action
        rospy.loginfo("Connecting to schunk_pullback...")
        self._schunk_pullback_client.wait_for_server()
        rospy.loginfo("...schunk_pullback connected")

    def run(self):
        """
        The run function for this step

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo("Action {}: Pullback gripper from schunk machine chuck".format(self.name))

        # Create and send the goal
        goal = SchunkPullbackGoal()
        self._schunk_pullback_client.send_goal(goal)

        # Yield an empty dict while we're executing
        while self._schunk_pullback_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Wait for a result and yield based on how we exited
        status = self._schunk_pullback_client.get_state()
        self._schunk_pullback_client.wait_for_result()
        result = self._schunk_pullback_client.get_result()

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
        self._schunk_pullback_client.cancel_goal()
