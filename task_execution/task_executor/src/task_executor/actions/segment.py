#!/usr/bin/env python
# Call segmentation

from __future__ import print_function, division

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from rail_manipulation_msgs.srv import SegmentObjects


class SegmentAction(AbstractStep):
    """
    Call rail_segmentation to segment the point cloud and return a list of
    segmented objects
    """

    SEGMENT_OBJECTS_SERVICE_NAME = "/rail_segmentation/segment_objects"

    def init(self, name):
        self.name = name

        # The segmentation interface
        self._segment_objects_srv = rospy.ServiceProxy(
            SegmentAction.SEGMENT_OBJECTS_SERVICE_NAME,
            SegmentObjects
        )

        # Set a stop flag
        self._stopped = False

        # Wait for a connection to rail_segmentation
        rospy.loginfo("Connecting to rail_segmentation...")
        self._segment_objects_srv.wait_for_service()
        rospy.loginfo("...rail_segmentation connected")

    def run(self, abort_on_zero=False):
        rospy.loginfo("Action {}: Segmenting objects".format(self.name))
        self._stopped = False

        # Ask for a list of segmented objects
        segmented_objects = self._segment_objects_srv().segmented_objects
        self.notify_service_called(SegmentAction.SEGMENT_OBJECTS_SERVICE_NAME)
        yield self.set_running()  # Check the status of the server

        if self._stopped:
            yield self.set_preempted(
                action=self.name,
                srv=self._segment_objects_srv.resolved_name,
                segmented_objects=segmented_objects
            )
        elif abort_on_zero and len(segmented_objects.objects) == 0:
            yield self.set_aborted(
                action=self.name,
                srv=self._segment_objects_srv.resolved_name,
                segmented_objects=segmented_objects
            )
        else:
            yield self.set_succeeded(segmented_objects=segmented_objects.objects)

    def stop(self):
        self._stopped = True
