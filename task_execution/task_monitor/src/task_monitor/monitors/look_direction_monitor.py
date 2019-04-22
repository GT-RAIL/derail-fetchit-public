#!/usr/bin/env python
# Monitor the goals sent to point head and check to see if the joint poses
# match the location. If not, signal a fault

from __future__ import print_function, division

import numpy as np

from threading import Lock

import rospy
import tf

from control_msgs.msg import PointHeadActionGoal
from sensor_msgs.msg import JointState

from task_monitor.monitoring import AbstractFaultMonitor


# The class definition

class LookDirectionMonitor(AbstractFaultMonitor):
    """
    Listen to goals sent to the point head controller and compare the current
    joint state to the desired state. If they don't match (within a tolerance)
    then signal a fault
    """

    LOOK_DIRECTION_MONITOR_EVENT_NAME = "look_direction_update"
    POINT_HEAD_GOAL_TOPIC = "/head_controller/point_head/goal"
    JOINT_STATES_TOPIC = "/joint_states"

    HEAD_PAN_JOINT = "head_pan_joint"
    HEAD_TILT_JOINT = "head_tilt_joint"
    HEAD_PAN_REF_FRAME = "base_link"
    HEAD_TILT_REF_FRAME = "head_pan_link"

    MONITOR_DURATION = rospy.Duration(0.3)  # Frequency of consistency checks
    JOINT_STATE_ERROR_BOUND = 0.05  # Approx 6 degress of error

    def __init__(self):
        super(LookDirectionMonitor, self).__init__()
        self.set_metadata(topics=[LookDirectionMonitor.POINT_HEAD_GOAL_TOPIC,
                                  LookDirectionMonitor.JOINT_STATES_TOPIC])

        # Initialize the tf listener
        self._listener = tf.TransformListener()

        # The joint states values
        self._last_pan_joint = None
        self._last_tilt_joint = None
        self._joint_lock = Lock()

        # The goal values
        self._last_pan_goal = None
        self._last_tilt_goal = None
        self._goal_lock = Lock()

        # Set up the subscribers
        self._joints_sub = rospy.Subscriber(
            LookDirectionMonitor.JOINT_STATES_TOPIC,
            JointState,
            self._on_joints
        )
        self._point_goal_sub = rospy.Subscriber(
            LookDirectionMonitor.POINT_HEAD_GOAL_TOPIC,
            PointHeadActionGoal,
            self._on_point_goal
        )

        # Setup the monitor timer
        self._monitor_timer = rospy.Timer(
            LookDirectionMonitor.MONITOR_DURATION,
            self._monitor_func,
            oneshot=False
        )

    def _on_joints(self, msg):
        locked = False
        try:
            if self._joint_lock.acquire(False):
                locked = True
                pan_joint_idx = msg.name.index(LookDirectionMonitor.HEAD_PAN_JOINT)
                tilt_joint_idx = msg.name.index(LookDirectionMonitor.HEAD_TILT_JOINT)

                self._last_pan_joint = msg.position[pan_joint_idx]
                self._last_tilt_joint = msg.position[tilt_joint_idx]
        finally:
            if locked:
                self._joint_lock.release()

    def _on_point_goal(self, msg):
        # Get the tf transforms of the goal in the head tilt and pan frames
        try:
            (trans_pan, _) = self._listener.lookupTransform(
                LookDirectionMonitor.HEAD_PAN_REF_FRAME,
                msg.goal.target.header.frame_id,
                rospy.Time(0)
            )

            (trans_tilt, _) = self._listener.lookupTransform(
                LookDirectionMonitor.HEAD_TILT_REF_FRAME,
                msg.goal.target.header.frame_id,
                rospy.Time(0)
            )
        except (tf.LookupException, tf.ExtrapolationException):
            return

        # Set the desired goal points (as tuples)
        pan_point = (
            msg.goal.target.point.x + trans_pan[0],
            msg.goal.target.point.y + trans_pan[1],
            msg.goal.target.point.z + trans_pan[2]
        )
        tilt_point = (
            msg.goal.target.point.x + trans_tilt[0],
            msg.goal.target.point.y + trans_tilt[1],
            msg.goal.target.point.z + trans_tilt[2]
        )

        # Finally, set the goal angles
        with self._goal_lock:
            self._last_pan_goal = np.arctan2(pan_point[1], pan_point[0])
            self._last_tilt_goal = np.arctan2(-tilt_point[2], tilt_point[0])

    def _monitor_func(self, evt):
        with self._joint_lock:
            with self._goal_lock:
                if self._last_pan_goal is None or self._last_tilt_goal is None \
                        or self._last_pan_joint is None or self._last_tilt_joint is None:
                    return None

                # Check if the joint is within bounds
                pan_within_bounds = \
                    np.abs(self._last_pan_goal - self._last_pan_joint) < LookDirectionMonitor.JOINT_STATE_ERROR_BOUND
                tilt_within_bounds = \
                    np.abs(self._last_tilt_goal - self._last_tilt_joint) < LookDirectionMonitor.JOINT_STATE_ERROR_BOUND

                # Send an updated trace
                return self.update_trace(
                    LookDirectionMonitor.LOOK_DIRECTION_MONITOR_EVENT_NAME,
                    not (pan_within_bounds and tilt_within_bounds),
                    { 'pan_goal': self._last_pan_goal, 'pan_joint': self._last_pan_joint,
                      'tilt_goal': self._last_tilt_goal, 'tilt_joint': self._last_tilt_joint }
                )


# When running the monitor in standalone mode
if __name__ == '__main__':
    rospy.init_node('look_direction_monitor')
    monitor = LookDirectionMonitor()
    rospy.spin()
