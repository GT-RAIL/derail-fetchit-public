#!/usr/bin/env python
# Create a point cloud diagnostic that reports an error if a new point cloud
# message has not been received in the last 1 sec

from __future__ import print_function, division

from threading import Lock

import rospy
import diagnostic_updater

from diagnostic_msgs.msg import DiagnosticStatus
from sensor_msgs.msg import PointCloud2


# The actual diagnostic class

class PointCloudDiagnostic(object):
    """Checks the point cloud and returns a fault if there hasn't been a point
    cloud in the last 1 sec"""

    POINT_CLOUD_TOPIC = "/head_camera/depth_registered/points"
    MAX_ALLOWED_DELAY = rospy.Duration(1.5)  # It's OK to not have a point cloud for this long

    def __init__(self):
        # Point cloud details
        self._last_pc_header = None
        self._pc_lock = Lock()
        self._pc_sub = rospy.Subscriber(PointCloudDiagnostic.POINT_CLOUD_TOPIC, PointCloud2, self._on_point_cloud)

        # Create the diagnostic updater
        self._updater = diagnostic_updater.Updater()
        self._updater.setHardwareID("none")
        self._updater.add("delay", self._create_diagnostic)

        # Set the loop rate of this updater (Hz)
        self._loop_rate = 20

    def _create_diagnostic(self, stat):
        with self._pc_lock:
            if self._last_pc_header is not None \
                    and rospy.Time.now() - self._last_pc_header.stamp <= PointCloudDiagnostic.MAX_ALLOWED_DELAY:
                status = DiagnosticStatus.OK
                message = "Point cloud delay is acceptable"
            else:
                status = DiagnosticStatus.ERROR
                message = "Point cloud is delayed"

        stat.summary(status, message)
        return stat

    def _on_point_cloud(self, msg):
        with self._pc_lock:
            self._last_pc_header = msg.header

    def spin(self):
        rate = rospy.Rate(self._loop_rate)
        while not rospy.is_shutdown():
            self._updater.update()
            rate.sleep()


# The main
if __name__ == '__main__':
    rospy.init_node("point_cloud_diagnostic")
    diag = PointCloudDiagnostic()
    diag.spin()
