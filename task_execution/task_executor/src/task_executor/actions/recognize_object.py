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
from task_execution_msgs.srv import GetPartsAtLocation, GetBeliefs, GetSemanticLocations


class RecognizeObjectAction(AbstractStep):
    """
    Given a list of point clouds from ``rail_segmentatsion``, recognize the
    desired ``manipulation_actions/ChallengeObject`` and return the most likely
    candidate point cloud in the list.
    """

    RECOGNIZE_OBJECT_SERVICE_NAME = "/rail_object_recognition/recognize_object"
    PARTS_AT_LOCATIONS_SERVICE_NAME = "/database/parts_at_location"
    SEMANTIC_LOCATIONS_SERVICE_NAME = "/database/semantic_locations"
    BELIEFS_SERVICE_NAME = "/beliefs/get_beliefs"
    EXPECTED_DISTANCE_SORT_FRAME = "base_link"

    # The indices of the challenge objects in the returned recognition output
    CHALLENGE_OBJECT_INDICES = {
        ChallengeObject.NONE: 5,
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


        # Postprocessing recognition results using semantic knowledge
        self._get_parts_at_location_srv = rospy.ServiceProxy(
            RecognizeObjectAction.PARTS_AT_LOCATIONS_SERVICE_NAME,
            GetPartsAtLocation
        )
        self._get_semantic_locations_srv = rospy.ServiceProxy(
            RecognizeObjectAction.SEMANTIC_LOCATIONS_SERVICE_NAME,
            GetSemanticLocations
        )
        self._get_beliefs_srv = rospy.ServiceProxy(
            RecognizeObjectAction.BELIEFS_SERVICE_NAME,
            GetBeliefs
        )

        rospy.loginfo("Connecting to database services...")
        self._get_semantic_locations_srv.wait_for_service()
        rospy.loginfo("...database services connected")

        rospy.loginfo("Connecting to database services...")
        self._get_parts_at_location_srv.wait_for_service()
        rospy.loginfo("...database services connected")

        rospy.loginfo("Connecting to belief services...")
        self._get_parts_at_location_srv.wait_for_service()
        rospy.loginfo("...belief services connected")

        self._parts_at_locations = {}
        self._get_parts_at_locations()

    def _get_parts_at_locations(self):
        locations = self._get_semantic_locations_srv().locations
        for location in locations:
            try:
                resp = self._get_parts_at_location_srv(location)
            except:
                continue

            if len(resp.parts_at_location.parts) > 0:
                parts = [p.object for p in resp.parts_at_location.parts]
                self._parts_at_locations[location] = parts

    def run(self, desired_obj, segmented_objects, checks={
        'check_location': True,
        'check_none_class': True,
        'check_threshold': 0.0,
        'sort_by_distance': False,
        'sort_by_centroid': False,
    }):
        """
        The run function for this step

        Args:
            desired_obj (manipulation_actions/ChallengeObject) : the type of
                challenge object that we wish to recognize
            segmented_objects (list of rail_manipulation_msgs/SegmentedObject) :
                a list of object point clouds, from
                :mod:`task_executor.actions.segment`, that we want to recognize
                the `desired_obj` in
            checks (dict) : a set of checks to perform. If set, we
                filter recognition results based on the results of the
                check. The available checks are:

                * ``check_location`` (default: true) - check that we are at the \
                    location where the desired object should be found
                * ``check_none_class`` (default: true) - check that the object \
                    we have recognized does not have a higher classification \
                    as NONE
                * ``check_threshold`` (default: 0.0) - check the \
                    classification confidence and do not use one that is below \
                    the specified threshold
                * ``sort_by_distance`` (default: false) - Sort the objects by \
                    their distance to the robot. Note the segmented objects \
                    MUST be in the :const:`EXPECTED_DISTANCE_SORT_FRAME` frame
                * ``sort_by_centroid`` (default: false) - Sort the objects by \
                    their distance from the centroid of all the other objects \
                    in their vicinity

        Yields:
            object_idx (int) : the index of the desired object in the input list

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """

        # We expect segmented_objects to be the output from `segment`
        rospy.loginfo("Action {}: Inspecting scene for object {}"
                      .format(self.name, desired_obj))

        # If the desired object is specified by its key, then disambiguate it
        if isinstance(desired_obj, str):
            desired_obj = getattr(ChallengeObject, desired_obj.upper())

        # Ensure that the args are valid
        assert desired_obj in RecognizeObjectAction.CHALLENGE_OBJECT_INDICES.keys(), \
            "Unknown desired object {}".format(desired_obj)

        # Use belief about current location to check if desired_object is valid
        if not self._pre_process(desired_obj, segmented_objects, checks):
            yield self.set_aborted(
                action=self.name,
                goal=desired_obj,
            )
            raise StopIteration()

        # Ensure there are segmented objects
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
        object_idx = self._post_process(classifications, desired_obj, segmented_objects, checks)
        if object_idx is None:
            yield self.set_aborted(
                action=self.name,
                goal=desired_obj
            )

        # For now, just stop
        rospy.loginfo("Action {}: Recognized object_idx is {}".format(self.name, object_idx))
        yield self.set_succeeded(object_idx=object_idx)

    def stop(self):
        self._stopped = True

    def _post_process(self, classifications, desired_obj, segmented_objects, checks):
        """
        Actually figure out which object we want based on the recognition flags
        that we have set
        """
        desired_col = RecognizeObjectAction.CHALLENGE_OBJECT_INDICES[desired_obj]

        # Get a sorted list of the objects. Lowest probability to highest
        best_object = None
        if checks.get('sort_by_distance') or checks.get('sort_by_centroid'):
            desired_rows = np.where(np.argmax(classifications, axis=1) == desired_col)[0]
            if len(desired_rows) == 0:
                rospy.loginfo("Action {}: desired class not most likely for any segmented object".format(self.name))
                return None

            if (
                checks.get('sort_by_distance')
                and segmented_objects[0].bounding_volume.pose.header.frame_id
                    == RecognizeObjectAction.EXPECTED_DISTANCE_SORT_FRAME
            ):
                rospy.loginfo("Action {}: Calculating distance weights".format(self.name))
                distance_weights = self._get_distance_weights([segmented_objects[i] for i in desired_rows])
            else:
                rospy.loginfo("Action {}: Not calculating distance weights".format(self.name))
                if (
                    segmented_objects[0].bounding_volume.pose.header.frame_id
                    != RecognizeObjectAction.EXPECTED_DISTANCE_SORT_FRAME
                ):
                    rospy.logwarn("Action {}: Unexpected frame {} for distance sort".format(
                        self.name,
                        segmented_objects[0].bounding_volume.pose.header.frame_id
                    ))
                distance_weights = np.ones_like(desired_rows, dtype=np.float)

            if checks.get('sort_by_centroid'):
                rospy.loginfo("Action {}: Calculating centroid weights".format(self.name))
                centroid_weights = self._get_centroid_weights([segmented_objects[i] for i in desired_rows])
            else:
                rospy.loginfo("Action {}: Not calculating centroid weights".format(self.name))
                centroid_weights = np.ones_like(desired_rows, dtype=np.float)

            weights = classifications[desired_rows, desired_col] * distance_weights * centroid_weights
            #sorted_objects = desired_rows[np.argsort(weights)]

            # sample best object with weighted probability
            sorting_indices_weights = np.argsort(weights)
            weights = weights[sorting_indices_weights]
            desired_rows = desired_rows[sorting_indices_weights]
            rospy.loginfo("Sorting weights for recognized objects {} are {}".format(desired_rows, weights))
            top3_desired_rows = desired_rows[:2]
            top3_weights = weights[:2] / np.sum(weights[:2])
            best_object = np.random.choice(top3_desired_rows, p=top3_weights)

        # Catch all if the previous sort post-processes do not apply
        if best_object is None:
            best_object = np.argsort(classifications[:, desired_col])[-1]

        # Make sure that the NONE class is not more likely
        if checks.get('check_none_class', True):
            none_col = RecognizeObjectAction.CHALLENGE_OBJECT_INDICES[ChallengeObject.NONE]
            if (classifications[best_object, none_col] > classifications[best_object, desired_col]):
                rospy.loginfo("Action {}: NONE class has better accuracy".format(self.name))
                return None

        # Make sure that the classification is above some threshold
        if checks.get('check_threshold', 0.0):
            threshold = checks.get('check_threshold', 0.0)
            if classifications[best_object, desired_col] < threshold:
                rospy.loginfo("Action {}: probability {} below threshold {}".format(
                    self.name,
                    classifications[best_object, desired_col],
                    threshold
                ))
                return None

        return best_object

    def _pre_process(self, desired_obj, segmented_objects, checks):
        """
        Make sure that the assumptions for correctly classifying are satisfied,
        such as the fact that the robot is at the right location
        """
        if checks.get('check_location', True):
            resp = self._get_beliefs_srv()
            belief_keys = resp.beliefs
            belief_values = resp.values
            current_location = None
            for key, value in zip(belief_keys, belief_values):
                if value == 0:
                    continue
                if "robot_at_" not in key:
                    continue
                location = key.replace("robot_at_", "").lower()
                if location not in self._parts_at_locations:
                    continue
                current_location = location
            rospy.loginfo("Action {}: Current location {}".format(self.name, current_location))

            if current_location is None or desired_obj not in self._parts_at_locations[current_location]:
                rospy.loginfo(
                    "Action {}: Current location {} doesn't have requested object {}"
                    .format(self.name, current_location, desired_obj)
                )
                return False

        # All checks have passed, return True
        return True

    def _get_distance_weights(self, segmented_objects):
        """
        Calculate the distance metric for each of the segmented_objects
        """
        distance = np.linalg.norm([
            [o.bounding_volume.pose.pose.position.x,
             o.bounding_volume.pose.pose.position.y,
             o.bounding_volume.pose.pose.position.z]
            for o in segmented_objects
        ], axis=1)

        # Catch the error case
        if np.all(distance == 0):
            return np.ones_like(distance, dtype=np.float)

        return 1 - (distance / np.amax(distance))

    def _get_centroid_weights(self, segmented_objects):
        """
        Calculate a closeness to centroid metric of all the segmented objects
        """
        positions = np.array([
            [o.bounding_volume.pose.pose.position.x,
             o.bounding_volume.pose.pose.position.y,
             o.bounding_volume.pose.pose.position.z]
            for o in segmented_objects
        ])
        centroid = np.mean(positions, axis=0)
        distance = np.linalg.norm(positions - centroid, axis=1)

        # Catch the error case
        if np.all(distance == 0):
            return np.ones_like(distance, dtype=np.float)

        return 1 - (distance / np.amax(distance))
