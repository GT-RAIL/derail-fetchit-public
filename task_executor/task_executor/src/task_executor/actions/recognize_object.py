#!/usr/bin/env python
# The find object action in a task plan

from __future__ import print_function, division

import numpy as np

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from rail_manipulation_msgs.msg import SegmentedObject
from rail_object_recognition.srv import ExtractPointCloud
from manipulation_actions.msg import ChallengeObject


class RecognizeObjectAction(AbstractStep):

    RECOGNIZE_OBJECT_SERVICE_NAME = "/rail_object_recognition/recognize_object"

    # The indices of the challenge objects in the returned recognition output
    CHALLENGE_OBJECT_INDICES = {
        ChallengeObject.BOLT: 4,
        ChallengeObject.SMALL_GEAR: 3,
        ChallengeObject.LARGE_GEAR: 2,
        ChallengeObject.GEARBOX_TOP: 0,
        ChallengeObject.GEARBOX_BOTTOM: 1,
    }

    def init(self, name):
        self.name = name

        # The object recognition API
        self._recognize_object_srv = rospy.ServiceProxy(
            RecognizeObjectAction.RECOGNIZE_OBJECT_SERVICE_NAME,
            ExtractPointCloud
        )

        # Set a stop flag
        self._stopped = False

        # Wait for a connection to the recognition API
        rospy.loginfo("Connecting to rail_object_recognition..")
        self._recognize_object_srv.wait_for_service()
        rospy.loginfo("...rail_object_recognition connected")

    def run(self, desired_obj, segmented_objects):
        # We expect segmented_objects to be the output from `segment`
        rospy.loginfo("Action {}: Inspecting scene for object {}"
                      .format(self.name, desired_obj))

        # If the desired object is specified by its key, then disambiguate it
        if isinstance(desired_obj, str):
            desired_obj = getattr(ChallengeObject, desired_obj.upper())

        # Ensure that the args are valid
        assert desired_obj in RecognizeObjectAction.CHALLENGE_OBJECT_INDICES.keys(), \
            "Unknown desired object {}".format(desired_obj)
        assert len(segmented_objects) > 0, "Cannot recognize wih 0 point clouds"
        self._stopped = False

        # Send the point clouds of the segmented objects
        classifications = self._recognize_object_srv(
            clouds=[obj.point_cloud for obj in segmented_objects]
        ).classes_of_parts
        self.notify_service_called(RecognizeObjectAction.RECOGNIZE_OBJECT_SERVICE_NAME)
        yield self.set_running()
        if self._stopped:
            yield self.set_preempted(
                action=self.name,
                goal=desired_obj,
                srv=self._recognize_object_srv.resolved_name,
                classification=classification
            )
            raise StopIteration()

        # Load the classifications and reshape them
        classifications = np.array(classifications).reshape(
            -1, len(RecognizeObjectAction.CHALLENGE_OBJECT_INDICES.keys())
        )

        # Then figure out the index of the desired object using max and argmax
        object_idx = np.argmax(
            classifications[:, RecognizeObjectAction.CHALLENGE_OBJECT_INDICES[desired_obj]],
        )

        # For now, just stop
        yield self.set_succeeded(object_idx=object_idx)

    def stop(self):
        self._stopped = True
