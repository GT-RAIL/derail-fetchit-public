#!/usr/bin/env python
# A service server that gives access to different data as needed

from __future__ import print_function, division

import rospy

from geometry_msgs.msg import PoseStamped
from task_execution_msgs.msg import Waypoint, Bounds, ObjectConstraints, ArmJointPose
from std_srvs.srv import Trigger, TriggerResponse
from task_execution_msgs.srv import (GetWaypoints, GetWaypointsResponse,
                                     GetObjectConstraints, GetObjectConstraintsResponse,
                                     GetArmGripperPose, GetArmGripperPoseResponse,
                                     GetArmJointPose, GetArmJointPoseResponse,
                                     GetTrajectory, GetTrajectoryResponse)


# The actual database node

class DatabaseServer(object):
    """
    Based on the ROS params that are loaded from a YAML file, this class
    provides a set of services that other nodes can use to query waypoints,
    arm trajectories, etc. by name
    """

    def __init__(self):
        # Provide a service to reload, then reload
        self._reload_service = rospy.Service('~reload', Trigger, self.reload)
        self.reload()

        # Start up the service servers for the different query types
        self._waypoints_service = rospy.Service(
            '~waypoints', GetWaypoints, self.get_waypoints
        )
        self._object_constraints_service = rospy.Service(
            '~object_constraints', GetObjectConstraints, self.get_object_constraints
        )
        self._arm_gripper_pose_service = rospy.Service(
            '~arm_gripper_pose', GetArmGripperPose, self.get_arm_gripper_pose
        )
        self._arm_joint_pose_service = rospy.Service(
            '~arm_joint_pose', GetArmJointPose, self.get_arm_joint_pose
        )
        self._trajectory_service = rospy.Service(
            '~trajectory', GetTrajectory, self.get_trajectory
        )

    def start(self):
        # This is a no-op at the moment
        rospy.loginfo("Database node ready...")

    def reload(self, req=None):
        # Validate the data in each of the expected rosparams and populate the
        # database
        self.waypoints = self._validate_waypoints(rospy.get_param('~waypoints', {}))
        self.object_constraints = self._validate_object_constraints(rospy.get_param('~object_constraints', {}))
        self.arm_gripper_poses = self._validate_arm_gripper_poses(rospy.get_param('~arm_gripper_poses', {}))
        self.arm_joint_poses = self._validate_arm_joint_poses(rospy.get_param('~arm_joint_poses', {}))
        self.trajectories = self._validate_trajectories(rospy.get_param('~trajectories', {}))

    def get_waypoints(self, req):
        resp = GetWaypointsResponse(waypoints=self.waypoints[req.name])
        return resp

    def get_object_constraints(self, req):
        resp = GetObjectConstraintsResponse(constraints=self.object_constraints[req.name])
        return resp

    def get_arm_gripper_pose(self, req):
        resp = GetArmGripperPoseResponse(pose=self.arm_gripper_poses[req.name])
        return resp

    def get_arm_joint_pose(self, req):
        resp = GetArmJointPoseResponse(pose=self.arm_joint_poses[req.name])
        return resp

    def get_trajectory(self, req):
        resp = GetTrajectoryResponse(trajectory=self.trajectories[req.name])
        return resp

    def _validate_waypoints(self, wp_defs):
        # Reload the waypoints
        waypoints = {}
        for name, wp_def in wp_defs.iteritems():
            waypoints[name] = [Waypoint(**x) for x in wp_def]

        return waypoints

    def _validate_object_constraints(self, oc_defs):
        # Reload the object constraints
        object_constraints = {}
        for name, oc_def in oc_defs.iteritems():
            object_constraints[name] = ObjectConstraints(
                use_bounds=oc_def.get('bounds') is not None,
                use_location=oc_def.get('location') is not None
            )

            if oc_def.get('bounds') is not None:
                object_constraints[name].bounds = Bounds(**oc_def['bounds'])

            if oc_def.get('location') is not None:
                object_constraints[name].location = Bounds(**oc_def['location'])

        return object_constraints

    def _validate_arm_gripper_poses(self, agp_defs):
        # Reload the arm gripper poses
        arm_gripper_poses = {}
        for name, agp_def in agp_defs.iteritems():
            arm_gripper_poses[name] = PoseStamped()
            arm_gripper_poses[name].header.frame_id = agp_def["frame"]
            arm_gripper_poses[name].pose.position.x = agp_def["position"]["x"]
            arm_gripper_poses[name].pose.position.y = agp_def["position"]["y"]
            arm_gripper_poses[name].pose.position.z = agp_def["position"]["z"]
            arm_gripper_poses[name].pose.orientation.x = agp_def["orientation"]["x"]
            arm_gripper_poses[name].pose.orientation.y = agp_def["orientation"]["y"]
            arm_gripper_poses[name].pose.orientation.z = agp_def["orientation"]["z"]
            arm_gripper_poses[name].pose.orientation.w = agp_def["orientation"]["w"]

        return arm_gripper_poses

    def _validate_arm_joint_poses(self, ajp_defs):
        # Reload the arm joint poses
        arm_joint_poses = {}
        for name, ap_def in ajp_defs.iteritems():
            arm_joint_poses[name] = ArmJointPose(angles=ap_def)

        return arm_joint_poses

    def _validate_trajectories(self, traj_defs):
        # Reload the trajectories
        trajectories = {}
        for name, traj_def in traj_defs.iteritems():
            trajectories[name] = [ArmJointPose(angles=x) for x in traj_def]

        return trajectories
