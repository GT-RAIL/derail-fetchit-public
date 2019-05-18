#!/usr/bin/env python
# Insert peg into the schunk machine

from __future__ import print_function, division

from threading import Thread

import rospy
import actionlib

import numpy as np

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from actionlib_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from task_executor.abstract_step import AbstractStep


class ArmFollowJointTrajAction(AbstractStep):
    """
    Sends a command to follow_joint_trajectory controller for the arm
    """

    ARM_FOLLOW_JOINT_TRAJ_SERVER = "/arm_controller/follow_joint_trajectory"

    def init(self, name):
        self.name = name
        self._arm_follow_joint_traj_client = actionlib.SimpleActionClient(
            ArmFollowJointTrajAction.ARM_FOLLOW_JOINT_TRAJ_SERVER,
            FollowJointTrajectoryAction
        )

        # Initialize the action server and the beliefs action
        rospy.loginfo("Connecting to _arm_follow_joint_traj...")
        self._arm_follow_joint_traj_client.wait_for_server()
        rospy.loginfo("..._arm_follow_joint_traj connected")

    def run(self, joint_pose, duration):
        """
        The run function for this step

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo("Sending command to arm_controller follow_joint_trajectory")

        # Create and send the goal
        goal = FollowJointTrajectoryGoal()
        trajectory = JointTrajectory()

        trajectory.points.append(JointTrajectoryPoint())

        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        trajectory.joint_names = joints
        
        trajectory.points[0].positions = joint_pose
#	trajectory.points[0].positions = [1.13, 0.61, 1.53, -1.10, 0.22, -1.86, -0.66]
        trajectory.points[0].velocities = [0.0 for _ in joint_pose]
        trajectory.points[0].accelerations = [0.0 for _ in joint_pose]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        goal.trajectory = trajectory

        self._arm_follow_joint_traj_client.send_goal(goal)

        # Yield an empty dict while we're executing
        while self._arm_follow_joint_traj_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Wait for a result and yield based on how we exited
        status = self._arm_follow_joint_traj_client.get_state()
        self._arm_follow_joint_traj_client.wait_for_result()
        result = self._arm_follow_joint_traj_client.get_result()

        err = np.linalg.norm(np.array(result.error.positions))
        if (err > 0.5):
            yield self.set_aborted(
                action=self.name,
                status=status,
                result=result,
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
        self._arm_follow_joint_traj_client.cancel_goal()
