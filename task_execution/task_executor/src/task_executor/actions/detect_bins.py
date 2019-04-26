#!/usr/bin/env python
# The action to detect bin poses and enter it into the tf tree

from __future__ import print_function, division

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from fetchit_bin_detector.srv import GetBinPose


class DetectBinsAction(AbstractStep):
    """
    If the camera is pointing at bins (kits), then call the service
    :const:`DETECT_BINS_SERVICE_NAME` to detect their poses. The service
    internally calls segmentation.
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

    def run(self, attach_collision_object=True, abort_on_zero=False):
        """
        The run function for this step

        Args:
            attach_collision_object (bool) : reattach the bin as a collision
                object to the base when it is detected
            abort_on_zero (bool) : abort and signal an error when no bins are
                detected

        Yields:
            bin_poses (list of geometry_msgs/PoseStamped) : poses of the bins

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo(
            "Action {}: Detecting bins. reattach: {}; abort-on-zero: {}".format(
                self.name,
                attach_collision_object,
                abort_on_zero
            )
        )
        self._stopped = False

        # Ask for the bin detector
        bin_poses = self._detect_bins_srv(attach_collision_object=attach_collision_object).bin_poses
        self.notify_service_called(DetectBinsAction.DETECT_BINS_SERVICE_NAME)
        yield self.set_running()  # Check the status of the server

        if self._stopped:
            yield self.set_preempted(
                action=self.name,
                srv=self._detect_bins_srv.resolved_name,
                bin_poses=bin_poses
            )
        elif abort_on_zero and len(bin_poses) == 0:
            rospy.loginfo("Action {}: Received {} bin poses".format(self.name, len(bin_poses)))
            yield self.set_aborted(
                action=self.name,
                srv=self._detect_bins_srv.resolved_name,
                bin_poses=bin_poses
            )
        else:
            yield self.set_succeeded(bin_poses=bin_poses)

    def stop(self):
        self._stopped = True
