#!/usr/bin/env python
# The action to detect bin poses and enter it into the tf tree

from __future__ import print_function, division

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from geometry_msgs.msg import Transform
from fetchit_icp.srv import TemplateMatch


class DetectSchunkAction(AbstractStep):
    """
    If the camera is pointing at the corner of the schunk machine and robot localized, call this service
    :const:`DETECT_SCHUNK_SERVICE_NAME` to detect the schunk machine's chuck pose. The service
    internally calls icp matching to register the matching template.
    """

    DETECT_SCHUNK_SERVICE_NAME = '/match_template'

    def init(self, name):
        self.name = name

        # The schunk detection interface
        self._detect_schunk_srv = rospy.ServiceProxy(
            DetectSchunkAction.DETECT_SCHUNK_SERVICE_NAME,
            TemplateMatch
        )

        # Set a stop flag
        self._stopped = False

        # Wait for the connection to the schunk detector
        rospy.loginfo("Connecting to detect_bins...")
        self._detect_schunk_srv.wait_for_service()
        rospy.loginfo("...detect_bins connected")

    def run(self):
        """
        The run function for this step

        Args:


        Yields:
            chuck_approach_pose (geometry_msgs/PoseStamped) : approach pose for impedance control with the chuck

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo(
            "Action {}: Detecting schunk.".format(
                self.name
            )
        )
        self._stopped = False

        # Ask for the schunk detector
        stub_tf = Transform()
        chuck_approach_pose = self._detect_schunk_srv(stub_tf).template_pose
        self.notify_service_called(DetectSchunkAction.DETECT_SCHUNK_SERVICE_NAME)
        yield self.set_running()  # Check the status of the server

        if self._stopped:
            yield self.set_preempted(
                action=self.name,
                srv=self._detect_schunk_srv.resolved_name,
                chuck_approach_pose=chuck_approach_pose
            )
        else:
            yield self.set_succeeded(chuck_approach_pose=chuck_approach_pose)

    def stop(self):
        self._stopped = True
