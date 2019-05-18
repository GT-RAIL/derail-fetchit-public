#!/usr/bin/env python
# Insert peg into the schunk machine

from __future__ import print_function, division

from threading import Thread

import rospy
import actionlib

import numpy as np

from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal)
from actionlib_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from task_execution_msgs.srv import GetArmJointPose

from task_executor.abstract_step import AbstractStep


class ArmFollowJointTrajAction(AbstractStep):
    """
    Sends a command to follow_joint_trajectory controller for the arm
    """

    ARM_FOLLOW_JOINT_TRAJ_SERVER = "/arm_controller/follow_joint_trajectory"
    ARM_JOINT_POSES_SERVICE_NAME = "/database/arm_joint_pose"
    JOINT_NAMES = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "upperarm_roll_joint",
        "elbow_flex_joint",
        "forearm_roll_joint",
        "wrist_flex_joint",
        "wrist_roll_joint",
    ]
    POSITION_ERROR_THRESHOLD = 0.5

    def init(self, name):
        self.name = name
        self._arm_follow_joint_traj_client = actionlib.SimpleActionClient(
            ArmFollowJointTrajAction.ARM_FOLLOW_JOINT_TRAJ_SERVER,
            FollowJointTrajectoryAction
        )
        self._get_arm_joint_pose_srv = rospy.ServiceProxy(
            ArmFollowJointTrajAction.ARM_JOINT_POSES_SERVICE_NAME,
            GetArmJointPose
        )

        # Initialize the action server and the beliefs action
        rospy.loginfo("Connecting to _arm_follow_joint_traj...")
        self._arm_follow_joint_traj_client.wait_for_server()
        rospy.loginfo("..._arm_follow_joint_traj connected")

        rospy.loginfo("Connecting to database services...")
        self._get_arm_joint_pose_srv.wait_for_service()
        rospy.loginfo("...database services connected")

    def run(self, joint_pose, duration=2.0):
        """
        The run function for this step

        Args:
            duration (float) : The amount of time within which to execute the
                trajectory
            joint_pose (str, list, tuple, dict) :
                The arm poses to move to. If the type is:

                * str. Then if the string starts with
                    * `joint_poses`, get a ``task_execution_msgs/ArmJointPose`` \
                        from :const:`ARM_JOINT_POSES_SERVICE_NAME` and move the \
                        joints to the desired pose
                * list, tuple. Then if the list is of
                    * `floats`, move the joints to the desired pose indicated \
                        with the float values

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        # Parse out the pose
        parsed_pose = self._parse_pose(joint_pose)
        if parsed_pose is None:
            rospy.logerr("Action {}: FAIL. Unknown Format: {}".format(self.name, joint_pose))
            raise KeyError(self.name, "Unknown Format", joint_pose)

        rospy.loginfo("Action {}: Moving arm to pose {}".format(self.name, parsed_pose))

        # Create and send the goal
        goal = FollowJointTrajectoryGoal()
        trajectory = JointTrajectory()

        trajectory.points.append(JointTrajectoryPoint())

        trajectory.joint_names = ArmFollowJointTrajAction.JOINT_NAMES

        trajectory.points[0].positions = joint_pose
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
        if err <= ArmFollowJointTrajAction.POSITION_ERROR_THRESHOLD and status == GoalStatus.SUCCEEDED:
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

    def _parse_poses(self, pose):
        """
        Parses out a meaningful set of poses from the incoming argument to the
        function.

        Params:
        - string. Startswith
            - joint_poses: get ArmJointPose from ARM_JOINT_POSES_SERVICE_NAME
        - list/tuple of list/tuples: Joint trajectory

        Returns:
        - [float, float, ...] of joint poses
        """
        parsed_pose = None
        if isinstance(pose, str):
            # This is a reference to stored pose in the DB
            db_name, pose = pose.split('.', 1)
            if db_name == 'joint_poses':
                parsed_pose = self._get_arm_joint_pose_srv(pose).pose.angles
        elif isinstance(pose, (list, tuple,)) \
                and len(pose) == len(ArmFollowJointTrajAction.JOINT_NAMES):
            parsed_pose = pose

        return parsed_pose
