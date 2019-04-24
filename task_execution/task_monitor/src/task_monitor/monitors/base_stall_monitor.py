#!/usr/bin/env python
# Monitor the commands sent to the root and the resulting movement from the
# robot

from __future__ import print_function, division

import numpy as np

from threading import Lock

import rospy

from task_execution_msgs.msg import MonitorMetadata
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from task_monitor.monitoring import AbstractFaultMonitor


# The class definition

class BaseStallMonitor(AbstractFaultMonitor):
    """
    Monitor the commands sent to the base and compare against the actual
    movement of the base. If there is a disconnect, flag the error
    """

    BASE_COMMAND_TOPIC = "/cmd_vel"
    ODOM_TOPIC = "/odom"
    BASE_STALL_MONITOR_EVENT_NAME = "base_stall_update"

    ZERO_VELOCITY_TOLERANCE = 0.009  # A value for velocity in odom signifying 0
    DETECTION_WAIT_DURATION = rospy.Duration(5.0)  # Number of seconds to wait before considering the robot stalled

    def __init__(self):
        super(BaseStallMonitor, self).__init__()
        self.set_metadata(topics=[BaseStallMonitor.BASE_COMMAND_TOPIC, BaseStallMonitor.ODOM_TOPIC])

        # Variables to help flag a stall
        self._last_cmd_vel = None
        self._cmd_lock = Lock()

        self.base_is_stalled = False
        self._last_stall_detection = None

        # Setup the subscribers
        self._base_cmd_sub = rospy.Subscriber(
            BaseStallMonitor.BASE_COMMAND_TOPIC,
            Twist,
            self._on_base_cmd
        )
        self._odom_sub = rospy.Subscriber(
            BaseStallMonitor.ODOM_TOPIC,
            Odometry,
            self._on_odom
        )

    def _on_base_cmd(self, cmd_msg):
        with self._cmd_lock:
            self._last_cmd_vel = cmd_msg

    def _on_odom(self, odom_msg):
        with self._cmd_lock:
            if self._last_cmd_vel is None:
                return None

            cmd_vel_zero = np.isclose(
                np.linalg.norm([self._last_cmd_vel.linear.x, self._last_cmd_vel.linear.y, self._last_cmd_vel.linear.z]),
                0.0,
                atol=BaseStallMonitor.ZERO_VELOCITY_TOLERANCE
            )
            cmd_vel_zero = cmd_vel_zero and np.isclose(
                np.linalg.norm([self._last_cmd_vel.angular.x, self._last_cmd_vel.angular.y, self._last_cmd_vel.angular.z]),
                0.0,
                atol=BaseStallMonitor.ZERO_VELOCITY_TOLERANCE
            )

        # Check to see if the odometry is reporting a zero
        odom_vel = odom_msg.twist.twist
        odom_vel_zero = np.isclose(
            np.linalg.norm([odom_vel.linear.x, odom_vel.linear.y, odom_vel.linear.z]),
            0.0,
            atol=BaseStallMonitor.ZERO_VELOCITY_TOLERANCE
        )
        odom_vel_zero = odom_vel_zero and np.isclose(
            np.linalg.norm([odom_vel.angular.x, odom_vel.angular.y, odom_vel.angular.z]),
            0.0,
            atol=BaseStallMonitor.ZERO_VELOCITY_TOLERANCE
        )

        # Now check to see if the timeout for the stall detection has passed
        trace_event = None  # The message that was published in the end
        base_is_stalled = bool(not cmd_vel_zero and odom_vel_zero)
        if base_is_stalled and (
            self._last_stall_detection is None
            or rospy.Time.now() < self._last_stall_detection + BaseStallMonitor.DETECTION_WAIT_DURATION
        ):
            if self._last_stall_detection is None:
                self._last_stall_detection = rospy.Time.now()
            return trace_event
        elif not base_is_stalled:
            self._last_stall_detection = None

        # It has been a sufficient amount of time. Update the stall flag
        self.base_is_stalled = base_is_stalled
        trace_event = self.update_trace(
            BaseStallMonitor.BASE_STALL_MONITOR_EVENT_NAME,
            self.base_is_stalled,
            { 'base_is_stalled': self.base_is_stalled,
              'stall_start_time': self._last_stall_detection }
        )

        return trace_event


# When running the monitor in standalone mode
if __name__ == '__main__':
    rospy.init_node('base_stall_monitor')
    monitor = BaseStallMonitor()
    rospy.spin()
