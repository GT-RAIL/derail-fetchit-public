#!/usr/bin/env python
# The look action in a task plan that changes the pan/tilt values

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from actionlib_msgs.msg import GoalStatus


class LookPanTiltAction(AbstractStep):

    JOINT_STATES_TOPIC = "/joint_states"
    HEAD_ACTION_SERVER = "/head_controller/follow_joint_trajectory"
    HEAD_ACTION_DURATION = 0.5  # The velocity limits in joystick_teleop suggest 2.1

    HEAD_PAN_JOINT_NAME = "head_pan_joint"
    HEAD_TILT_JOINT_NAME = "head_tilt_joint"

    HEAD_PAN_MAX = 1.57
    HEAD_PAN_MIN = -1.57
    HEAD_TILT_MAX = 1.45
    HEAD_TILT_MIN = -0.76

    def init(self, name):
        self.name = name
        self._head_client = actionlib.SimpleActionClient(
            LookPanTiltAction.HEAD_ACTION_SERVER,
            FollowJointTrajectoryAction
        )
        self._joint_names = [LookPanTiltAction.HEAD_PAN_JOINT_NAME, LookPanTiltAction.HEAD_TILT_JOINT_NAME]
        self._duration = LookPanTiltAction.HEAD_ACTION_DURATION

        rospy.loginfo("Connecting to head_controller...")
        self._head_client.wait_for_server()
        rospy.loginfo("...head_controller connected")

        # Create a subscriber to check if we even have to move to the desired
        # torso height
        self._current_pan = 0.0
        self._current_tilt = 0.0
        self._joints_sub = rospy.Subscriber(
            LookPanTiltAction.JOINT_STATES_TOPIC,
            JointState,
            self._on_joints
        )

    def run(self, pan_amount=0.0, tilt_amount=0.0):
        rospy.loginfo("Action {}: Pan by amount {}, Tilt by amount {}".format(self.name, pan_amount, tilt_amount))

        # Update the desired pan and tilt positions based on the joint limits
        desired_pan = max(LookPanTiltAction.HEAD_PAN_MIN, self._current_pan + pan_amount)
        desired_pan = min(LookPanTiltAction.HEAD_PAN_MAX, desired_pan)
        desired_tilt = max(LookPanTiltAction.HEAD_TILT_MIN, self._current_tilt + tilt_amount)
        desired_tilt = min(LookPanTiltAction.HEAD_TILT_MAX, desired_tilt)

        # Create and send the goal height
        trajectory = JointTrajectory()
        trajectory.joint_names = self._joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = [desired_pan, desired_tilt]
        trajectory.points[0].velocities = [0.0, 0.0]
        trajectory.points[0].accelerations = [0.0, 0.0]
        trajectory.points[0].time_from_start = rospy.Duration(self._duration)

        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self._head_client.send_goal(follow_goal)
        self.notify_action_send_goal(LookPanTiltAction.HEAD_ACTION_SERVER, follow_goal)

        # Yield an empty dict while we're executing
        while self._head_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Wait for a result and yield based on how we exited
        status = self._head_client.get_state()
        self._head_client.wait_for_result()
        result = self._head_client.get_result()
        self.notify_action_recv_result(LookPanTiltAction.HEAD_ACTION_SERVER, status, result)

        if status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted(
                action=self.name,
                status=status,
                goal=[desired_pan, desired_tilt],
                result=result
            )
        else:
            yield self.set_aborted(
                action=self.name,
                status=status,
                goal=[desired_pan, desired_tilt],
                result=result
            )

    def stop(self):
        self._head_client.cancel_goal()
        self.notify_action_cancel(LookPanTiltAction.HEAD_ACTION_SERVER)

    def _on_joints(self, msg):
        try:
            idx = msg.name.index(LookPanTiltAction.HEAD_PAN_JOINT_NAME)
            self._current_pan = msg.position[idx]
            idx = msg.name.index(LookPanTiltAction.HEAD_TILT_JOINT_NAME)
            self._current_tilt = msg.position[idx]
        except ValueError as e:
            pass
