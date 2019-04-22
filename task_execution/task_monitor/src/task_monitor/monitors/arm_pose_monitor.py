#!/usr/bin/env python
# Arm pose monitor

from __future__ import print_function, division

import numpy as np

import rospy

from sensor_msgs.msg import JointState
from task_execution_msgs.msg import BeliefKeys, ArmJointPose
from task_execution_msgs.srv import GetArmJointPose

from task_monitor.monitoring import AbstractBeliefMonitor


# The class definition

class ArmPoseMonitor(AbstractBeliefMonitor):
    """
    Monitor the joint poses of the arm and update the belief of the pose that
    the arm is currently in
    """

    ARM_JOINT_NAMES = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "upperarm_roll_joint",
        "elbow_flex_joint",
        "forearm_roll_joint",
        "wrist_flex_joint",
        "wrist_roll_joint",
    ]
    JOINT_STATES_TOPIC = "/joint_states"
    JOINT_STATE_TOLERANCE = 0.38  # Approx ~ sqrt(7 * pi/150). pi/150 ~ 1.2 degrees

    ARM_JOINT_POSES_SERVICE_NAME = "/database/arm_joint_pose"
    TUCK_POSE_NAME = "tuck"
    STOW_POSE_NAME = "stow"
    READY_POSE_NAME = "ready"

    def __init__(self):
        super(ArmPoseMonitor, self).__init__()

        # Connect to the database service and get the poses
        get_arm_poses_srv = rospy.ServiceProxy(ArmPoseMonitor.ARM_JOINT_POSES_SERVICE_NAME, GetArmJointPose)
        rospy.loginfo("Connecting to database services...")
        get_arm_poses_srv.wait_for_service()
        rospy.loginfo("...database services connected")

        self._pose_belief_checks = {
            BeliefKeys.ARM_AT_TUCK: np.array(get_arm_poses_srv(ArmPoseMonitor.TUCK_POSE_NAME).pose.angles),
            BeliefKeys.ARM_AT_STOW: np.array(get_arm_poses_srv(ArmPoseMonitor.STOW_POSE_NAME).pose.angles),
            BeliefKeys.ARM_AT_READY: np.array(get_arm_poses_srv(ArmPoseMonitor.READY_POSE_NAME).pose.angles),
        }

        # Setup the monitor
        self._joint_states_sub = rospy.Subscriber(
            ArmPoseMonitor.JOINT_STATES_TOPIC,
            JointState,
            self._on_joint_states
        )

    def _on_joint_states(self, msg):
        try:
            idx = msg.name.index(ArmPoseMonitor.ARM_JOINT_NAMES[0])
            arm_pose = np.array(msg.position[idx:idx+len(ArmPoseMonitor.ARM_JOINT_NAMES)])

            beliefs = {
                belief: (np.linalg.norm(arm_pose - belief_pose) <= ArmPoseMonitor.JOINT_STATE_TOLERANCE)
                for belief, belief_pose in self._pose_belief_checks.iteritems()
            }

            # Return the messages that were sent
            return self.update_beliefs(beliefs)
        except ValueError as e:
            pass


# When running the monitor in standalone mode
if __name__ == '__main__':
    rospy.init_node('arm_pose_monitor')
    monitor = ArmPoseMonitor()
    rospy.spin()
