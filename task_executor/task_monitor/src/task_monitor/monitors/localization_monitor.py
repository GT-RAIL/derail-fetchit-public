#!/usr/bin/env python
# Monitor the localization of the robot and send out an event if it updates by
# more than a threshold within some time

from __future__ import print_function, division

import numpy as np

import rospy
import tf

from task_execution_msgs.msg import MonitorMetadata
from nav_msgs.msg import Odometry

from task_monitor.monitoring import AbstractFaultMonitor


# The class definition

class LocalizationMonitor(AbstractFaultMonitor):
    """
    Monitors tf and sends out an alert event if the localization of the robot
    updates by more than some pre-specified threshold.
    """

    LOCALIZATION_MONITOR_EVENT_NAME = "localization_update"
    LOCALIZATION_MONITOR_NODE = "/amcl"

    MONITORING_LOOP_RATE = 10  # Hz
    MONITORING_LINEAR_ERROR_BOUND = 0.03  # 3 cm bounds on the error between checks
    MONITORING_ANGULAR_ERROR_BOUND = 0.05  # Approx 6 degrees of error between checks

    MAP_FRAME = "/map"
    BASE_FRAME = "/base_link"
    ODOM_TOPIC = "/odom"

    def __init__(self):
        super(LocalizationMonitor, self).__init__()
        self.set_metadata(nodes=[LocalizationMonitor.LOCALIZATION_MONITOR_NODE])

        # Initialize the tf listener
        self._listener = tf.TransformListener()

        # Get the odometry
        self._last_vel = None
        self._odom_sub = rospy.Subscriber(LocalizationMonitor.ODOM_TOPIC, Odometry, self._on_odom)

        # Stub values to save the last check
        self._last_position = np.zeros((3,))
        self._last_orientation = np.zeros((4,))

        # Start the monitor
        self._monitor_timer = rospy.Timer(
            rospy.Duration(1 / LocalizationMonitor.MONITORING_LOOP_RATE),
            self._monitor_func,
            oneshot=False
        )

    def _on_odom(self, odom_msg):
        self._last_vel = odom_msg.twist.twist

    def _monitor_func(self, evt):
        # First try to lookup the transform and convert to numpy arrays
        try:
            (trans, rot) = self._listener.lookupTransform(
                LocalizationMonitor.MAP_FRAME,
                LocalizationMonitor.BASE_FRAME,
                rospy.Time(0)
            )
            trans, rot = np.array(trans), np.array(rot)
        except (tf.LookupException, tf.ExtrapolationException):
            return None

        # Check the limits from the odom reported values
        max_bound_linear = (
            self._last_vel.linear.x / LocalizationMonitor.MONITORING_LOOP_RATE
            + LocalizationMonitor.MONITORING_LINEAR_ERROR_BOUND
        )
        max_bound_angular = (
            self._last_vel.angular.z / LocalizationMonitor.MONITORING_LOOP_RATE
            + LocalizationMonitor.MONITORING_ANGULAR_ERROR_BOUND
        )

        # Check the bounds on the localization update
        is_outside_bounds = False
        diff_linear = np.linalg.norm(trans - self._last_position)
        diff_angular = (1 - np.dot(rot, self._last_orientation) ** 2)
        if diff_linear >= max_bound_linear:
            is_outside_bounds = True
        if diff_angular >= max_bound_angular:
            is_outside_bounds = True

        # Send an event update
        trace_event = self.update_trace(
            LocalizationMonitor.LOCALIZATION_MONITOR_EVENT_NAME,
            is_outside_bounds,
            { 'diff_linear': diff_linear, 'diff_angular': diff_angular }
        )

        # Update the values of the saved pose
        self._last_position = trans
        self._last_orientation = rot

        # Return the message that was sent
        return trace_event


# When running the monitor in standalone mode
if __name__ == '__main__':
    rospy.init_node("localization_monitor")
    monitor = LocalizationMonitor()
    rospy.spin()
