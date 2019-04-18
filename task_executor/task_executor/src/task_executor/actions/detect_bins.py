#!/usr/bin/env python
# The action to detect bin poses and enter it into the tf tree

from __future__ import print_function, division

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from fetchit_bin_detector.srv import GetBinPose


class DetectBinsAction(AbstractStep):
    """
    If the camera is pointing at bins (kits), then detect their pose
    """

    DETECT_BINS_SERVICE_NAME = '/detect_bins'

    def init(self, name):
        self.name = name

        # The bin detection interface
        self._detect_bins_srv = rospy.ServiceProxy(
            DetectBinsAction.DETECT_BINS_SERVICE_NAME,
            GetBinPose
        )

        # Set a stop flag
        self._stopped = False

        # Wait for the connection to the bin detector
        rospy.loginfo("Connecting to detect_bins...")
        self._detect_bins_srv.wait_for_service()
        rospy.loginfo("...detect_bins connected")

    def run(self, abort_on_zero=False):
        rospy.loginfo("Action {}: Detecting bins".format(self.name))
        self._stopped = False

        # Ask for the bin detector
        bin_poses = self._detect_bins_srv().bin_poses
        self.notify_service_called(DetectBinsAction.DETECT_BINS_SERVICE_NAME)
        yield self.set_running()  # Check the status of the server
        print(bin_poses)

        if self._stopped:
            yield self.set_preempted(
                action=self.name,
                srv=self._detect_bins_srv.resolved_name,
                bin_poses=bin_poses
            )
        elif abort_on_zero and len(bin_poses) == 0:
            yield self.set_aborted(
                action=self.name,
                srv=self._detect_bins_srv.resolved_name,
                bin_poses=bin_poses
            )
        else:
            yield self.set_succeeded(bin_poses=bin_poses)

    def stop(self):
        self._stopped = True
