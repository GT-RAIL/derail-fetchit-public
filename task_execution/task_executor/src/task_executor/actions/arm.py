#!/usr/bin/env python
# The arm pose action in a task plan

from __future__ import print_function, division

import sys

import rospy
import actionlib
import moveit_commander

from task_executor.abstract_step import AbstractStep

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from fetch_grasp_suggestion.msg import PresetJointsMoveAction, PresetJointsMoveGoal
from task_execution_msgs.msg import ArmJointPose
from task_execution_msgs.srv import GetArmGripperPose, GetArmJointPose, GetTrajectory

from .look_at_gripper import LookAtGripperAction


class ArmAction(AbstractStep):

    MAX_ATTEMPTS = 5
    JOINT_POSE_ACTION_SERVER = "/grasp_executor/preset_position"
    MOVE_GROUP_PLAN_ACTION_SERVER = "/move_group"
    MOVE_GROUP_EXECUTE_SERVICE = "/execute_kinematic_path"

    ARM_GRIPPER_POSES_SERVICE_NAME = "/database/arm_gripper_pose"
    ARM_JOINT_POSES_SERVICE_NAME = "/database/arm_joint_pose"
    TRAJECTORIES_SERVICE_NAME = "/database/trajectory"

    ARM_JOINT_NAMES = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "upperarm_roll_joint",
        "elbow_flex_joint",
        "forearm_roll_joint",
        "wrist_flex_joint",
        "wrist_roll_joint",
    ]
    ARM_GROUP_NAME = "arm"
    ARM_PLANNER_NAME = "arm[RRTConnectkConfigDefault]"
    ARM_PLANNING_TIME = 1.5
    ARM_EEF_FRAME = "wrist_roll_link"

    def init(self, name):
        self.name = name

        # Initialize the moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the service clients, action clients, etc.
        self._joint_pose_client = actionlib.SimpleActionClient(
            ArmAction.JOINT_POSE_ACTION_SERVER,
            PresetJointsMoveAction
        )
        self._move_group = moveit_commander.MoveGroupCommander(ArmAction.ARM_GROUP_NAME)
        self._get_arm_gripper_pose_srv = rospy.ServiceProxy(ArmAction.ARM_GRIPPER_POSES_SERVICE_NAME, GetArmGripperPose)
        self._get_arm_joint_pose_srv = rospy.ServiceProxy(ArmAction.ARM_JOINT_POSES_SERVICE_NAME, GetArmJointPose)
        self._get_trajectory_srv = rospy.ServiceProxy(ArmAction.TRAJECTORIES_SERVICE_NAME, GetTrajectory)

        # Set some execution flags
        self._stopped = False
        self._max_attempts = ArmAction.MAX_ATTEMPTS

        # Initialize the look at gripper module
        self._look_at_gripper = LookAtGripperAction()

        # Connect to the various services
        rospy.loginfo("Connecting to arm_preset_pose_executor...")
        self._joint_pose_client.wait_for_server()
        rospy.loginfo("...arm_preset_pose_executor connected")

        rospy.loginfo("Connecting to database services...")
        self._get_arm_gripper_pose_srv.wait_for_service()
        self._get_arm_joint_pose_srv.wait_for_service()
        self._get_trajectory_srv.wait_for_service()
        rospy.loginfo("...database services connected")

        self._look_at_gripper.init('look_at_gripper_arm')

    def run(self, poses, max_velocity_scaling=0.3, look_at_gripper=False):
        # Parse out the poses
        parsed_poses = self._parse_poses(poses)
        if parsed_poses is None:
            rospy.logerr("Action {}: FAIL. Unknown Format: {}".format(self.name, poses))
            raise KeyError(self.name, "Unknown Format", poses)

        rospy.logdebug("Action {}: Moving to arm pose(s): {}".format(self.name, parsed_poses))

        try:
            # Enable the look at gripper behaviour
            if look_at_gripper:
                self._look_at_gripper(enable=True)
                rospy.sleep(0.5)

            # Yield results based on the type of execution being run
            if isinstance(parsed_poses[0], ArmJointPose):
                for variables in self._run_joint_poses(parsed_poses, max_velocity_scaling):
                    yield variables
            elif isinstance(parsed_poses[0], PoseStamped):
                for variables in self._run_gripper_poses(parsed_poses, max_velocity_scaling):
                    yield variables
            else:
                # This should never happen
                raise KeyError("Unknown poses parsed from arguments: {}".format(parsed_poses))
        finally:
            # Stop looking at the gripper, if we were doing that
            if look_at_gripper:
                self._look_at_gripper(enable=False)
                rospy.sleep(0.5)

    def _run_gripper_poses(self, parsed_poses, max_velocity_scaling):
        # Run the arm pose follower on a series of PoseStamped
        self._stopped = False
        attempt_num = -1
        success = False
        for pose in parsed_poses:
            rospy.loginfo("Action {}: Going to gripper pose: {}".format(
                self.name,
                { "frame": pose.header.frame_id,
                  "position": str(pose.pose.position).replace("\n", ", "),
                  "orientation": str(pose.pose.orientation).replace("\n", ", ") }
            ))

            # Configure the move group planners
            self._move_group.set_max_velocity_scaling_factor(max_velocity_scaling)
            self._move_group.set_planner_id(ArmAction.ARM_PLANNER_NAME)
            self._move_group.set_planning_time(ArmAction.ARM_PLANNING_TIME)

            # Try to plan and execute
            for attempt_num in xrange(self._max_attempts):
                # Stop any arm motion and clear previous targets
                self._move_group.stop()
                self._move_group.clear_pose_targets()

                # Create and set the new pose target
                pose.header.stamp = rospy.Time.now()
                self._move_group.set_start_state_to_current_state()
                self._move_group.set_pose_target(pose, ArmAction.ARM_EEF_FRAME)

                # Then plan
                self.notify_action_send_goal(ArmAction.MOVE_GROUP_PLAN_ACTION_SERVER, pose)
                plan = self._move_group.plan()
                self.notify_action_recv_result(ArmAction.MOVE_GROUP_PLAN_ACTION_SERVER, GoalStatus.SUCCEEDED, plan)

                yield self.set_running()  # Check the status of the server
                if self._stopped:
                    yield self.set_preempted(
                        action=self.name,
                        goal=pose,
                        attempt_num=attempt_num
                    )
                    raise StopIteration()

                # Then move
                success = self._move_group.execute(plan, wait=True)
                self.notify_service_called(ArmAction.MOVE_GROUP_EXECUTE_SERVICE)

                yield self.set_running()  # Check the status of the server
                if self._stopped:
                    yield self.set_preempted(
                        action=self.name,
                        goal=pose,
                        attempt_num=attempt_num
                    )
                    raise StopIteration()

                # If this was successful, then break out
                if success:
                    break

                # Wait a bit before retrying
                rospy.sleep(0.5)

            # If reaching the previous pose was not successful, then break out
            # Otherwise, move on to the next pose
            if not success:
                break

        # Yield based on how we exited. Preempts are handled earlier
        if success:
            yield self.set_succeeded()
        else:
            yield self.set_aborted(
                action=self.name,
                goal=pose,
                attempt_num=attempt_num
            )

    def _run_joint_poses(self, parsed_poses, max_velocity_scaling):
        # Run the arm pose follower on a series of joint poses
        status = GoalStatus.LOST
        attempt_num = -1
        for pose in parsed_poses:
            rospy.loginfo("Action {}: Going to arm joint pose: {}".format(self.name, pose.angles))

            # Create and send the goal
            goal = PresetJointsMoveGoal()
            goal.name.extend(ArmAction.ARM_JOINT_NAMES)
            goal.position = pose.angles
            goal.max_velocity_scaling_factor = max_velocity_scaling
            assert len(goal.name) == len(goal.position)

            for attempt_num in xrange(self._max_attempts):
                rospy.loginfo("Action {}: Attempt {}/{}".format(self.name, attempt_num + 1, self._max_attempts))
                self._joint_pose_client.send_goal(goal)
                self.notify_action_send_goal(ArmAction.JOINT_POSE_ACTION_SERVER, goal)

                # Yield running while the client is executing
                while self._joint_pose_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
                    yield self.set_running()

                # Yield based on the server's status
                status = self._joint_pose_client.get_state()
                self._joint_pose_client.wait_for_result()
                result = self._joint_pose_client.get_result()
                self.notify_action_recv_result(ArmAction.JOINT_POSE_ACTION_SERVER, status, result)

                # Exit if we have succeeded or been preempted
                if status == GoalStatus.SUCCEEDED or status == GoalStatus.PREEMPTED:
                    break

                # Wait a bit before retrying
                rospy.sleep(0.5)

            # If we haven't succeeded in reaching this intermediate pose, then
            # break. Otherwise, move on to the next pose
            if status != GoalStatus.SUCCEEDED:
                break

        # Yield based on how we exited
        if status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted(
                action=self.name,
                status=status,
                goal=pose,
                attempt_num=attempt_num,
                result=result
            )
        else:
            yield self.set_aborted(
                action=self.name,
                status=status,
                goal=pose,
                attempt_num=attempt_num,
                result=result
            )

    def stop(self):
        self._stopped = True
        self._look_at_gripper.stop()
        self._joint_pose_client.cancel_goal()
        self.notify_action_cancel(ArmAction.JOINT_POSE_ACTION_SERVER)
        self._move_group.stop()

    def _parse_poses(self, poses):
        """
        Parses out a meaningful set of poses from the incoming argument to the
        function.

        Params:
        - string. Startswith
            - gripper_poses: get PoseStamped from ARM_GRIPPER_POSES_SERVICE_NAME
            - joint_poses: get ArmJointPose from ARM_JOINT_POSES_SERVICE_NAME
            - trajectories: get [ArmJointPose, ...] from TRAJECTORIES_SERVICE_NAME
        - list/tuple of list/tuples: Joint trajectory
        - list/tuple of floats: ArmJointPose
        - dictionary with the keys position, orientation, & frame: PoseStamped

        Returns:
        - [ArmJointPose(), ...] if the poses are joint poses
        - [PoseStamped(), ...] if the poses are EEF poses
        """
        parsed_poses = None

        if isinstance(poses, str):
            # This is a reference to stored poses in the DB
            db_name, poses = poses.split('.', 1)
            if db_name == 'gripper_poses':
                parsed_poses = [self._get_arm_gripper_pose_srv(poses).pose,]
                self.notify_service_called(ArmAction.ARM_GRIPPER_POSES_SERVICE_NAME)
            elif db_name == 'joint_poses':
                parsed_poses = [self._get_arm_joint_pose_srv(poses).pose,]
                self.notify_service_called(ArmAction.ARM_JOINT_POSES_SERVICE_NAME)
            elif db_name == 'trajectories':
                parsed_poses = self._get_trajectory_srv(poses).trajectory
                self.notify_service_called(ArmAction.TRAJECTORIES_SERVICE_NAME)
        elif isinstance(poses, (list, tuple,)) and len(poses) > 0 and isinstance(poses[0], (list, tuple,)):
            # YAML definition of a trajectory
            parsed_poses = [ArmJointPose(angles=x) for x in poses]
        elif isinstance(poses, (list, tuple,)) and len(poses) > 0:
            # YAML definition of a pose
            parsed_poses = [ArmJointPose(angles=poses),]
        elif isinstance(poses, dict) \
                and poses.has_key("position") and poses.has_key("orientation") and poses.has_key("frame"):
            # YAML definition of a PoseStamped EEF pose
            parsed_poses = [PoseStamped(),]
            parsed_poses[0].header.frame_id = poses['frame']
            parsed_poses[0].pose.position.x = poses['position']['x']
            parsed_poses[0].pose.position.y = poses['position']['y']
            parsed_poses[0].pose.position.z = poses['position']['z']
            parsed_poses[0].pose.orientation.x = poses['orientation']['x']
            parsed_poses[0].pose.orientation.y = poses['orientation']['y']
            parsed_poses[0].pose.orientation.z = poses['orientation']['z']
            parsed_poses[0].pose.orientation.w = poses['orientation']['w']

        return parsed_poses
