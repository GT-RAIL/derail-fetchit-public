#!/usr/bin/env python
# Arm approach schunk machine chuck

from __future__ import print_function

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from manipulation_actions.msg import (SchunkDoorAction as SchunkDoorMsg,
                                      SchunkDoorGoal)
from actionlib_msgs.msg import GoalStatus


class SchunkDoorAction(AbstractStep):
    """
    makes the arm open/close the schunk machine door
    """

    SCHUNK_DOOR_ACTION_SERVER = '/schunk_door_node/operate_door'

    def init(self, name):
        self.name = name
        self._door_client = actionlib.SimpleActionClient(
            SchunkDoorAction.SCHUNK_DOOR_ACTION_SERVER,
            SchunkDoorMsg
        )

        rospy.loginfo("Connecting to schunk_door_node...")
        self._door_client.wait_for_server()
        rospy.loginfo("...schunk_door_node connected")

    def run(self, approach_transform, command):
        """
        The run function for this step

        Args:
            approach_transform (geometry_msgs/TransformStamped) : approach pose from schunk detection
            command (str) : "open" or "close" the schunk door

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        if not isinstance(command, str) or command.lower() not in ['close', 'open']:
            rospy.logerr("Action: {}. FAIL. Unrecognized: {}".format(self.name, command))
            raise KeyError(self.name, "Unrecognized", command)

        rospy.loginfo("Action {}: Schunk door {}".format(self.name, command))

        # Create and send the goal
        goal = SchunkDoorGoal()
        goal.approach_transform = approach_transform
        if command.lower() == "open":
            goal.open = 1
        elif command.lower() == "close":
            goal.open = 0

        self._door_client.send_goal(goal)
        self.notify_action_send_goal(
            SchunkDoorAction.SCHUNK_DOOR_ACTION_SERVER, goal
        )

        # Yield while we're executing
        while self._door_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Wait for a result and yield based on how we exited
        status = self._door_client.get_state()
        self._door_client.wait_for_result()
        result = self._door_client.get_result()
        self.notify_action_recv_result(
            SchunkDoorAction.SCHUNK_DOOR_ACTION_SERVER, status, result
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
        self._door_client.cancel_goal()
        self.notify_action_cancel(SchunkDoorAction.SCHUNK_DOOR_ACTION_SERVER)
