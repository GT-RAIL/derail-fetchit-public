#!/usr/bin/env python
# The gripper action in a task plan

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from actionlib_msgs.msg import GoalStatus


class GripperAction(AbstractStep):

    GRIPPER_ACTION_SERVER = "/gripper_controller/gripper_action"
    GRIPPER_MAX_EFFORT = 200
    GRIPPER_OPEN_POSITION = 0.15
    GRIPPER_CLOSE_POSITION = 0.0

    def init(self, name):
        self.name = name
        self._gripper_client = actionlib.SimpleActionClient(
            GripperAction.GRIPPER_ACTION_SERVER,
            GripperCommandAction
        )

        rospy.loginfo("Connecting to gripper_controller...")
        self._gripper_client.wait_for_server()
        rospy.loginfo("...gripper_controller connected")

    def run(self, command):
        if not isinstance(command, str) or command.lower() not in ['close', 'open']:
            rospy.logerr("Action: {}. FAIL. Unrecognized: {}".format(self.name, command))
            raise KeyError(self.name, "Unrecognized", command)

        rospy.loginfo("Action {}: Gripper {}".format(self.name, command))

        # Create and send the goal pose
        goal = GripperCommandGoal()
        if command.lower() == 'close':
            goal.command.position = GripperAction.GRIPPER_CLOSE_POSITION
            goal.command.max_effort = GripperAction.GRIPPER_MAX_EFFORT
        elif command.lower() == 'open':
            goal.command.position = GripperAction.GRIPPER_OPEN_POSITION

        self._gripper_client.send_goal(goal)
        self.notify_action_send_goal(GripperAction.GRIPPER_ACTION_SERVER, goal)

        # Yield an empty dict while we're executing
        while self._gripper_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Wait for a result and yield based on how we exited
        status = self._gripper_client.get_state()
        self._gripper_client.wait_for_result()
        result = self._gripper_client.get_result()
        self.notify_action_recv_result(GripperAction.GRIPPER_ACTION_SERVER, status, result)

        if status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted(
                action=self.name,
                status=status,
                goal=command,
                result=result
            )
        else:
            yield self.set_aborted(
                action=self.name,
                status=status,
                goal=command,
                result=result
            )

    def stop(self):
        self._gripper_client.cancel_goal()
        self.notify_action_cancel(GripperAction.GRIPPER_ACTION_SERVER)
