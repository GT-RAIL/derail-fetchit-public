#!/usr/bin/env python
# The find object action in a task plan

from __future__ import print_function, division

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from std_srvs.srv import Empty
from rail_manipulation_msgs.msg import SegmentedObjectList
from rail_manipulation_msgs.srv import SegmentObjects
from fetch_grasp_suggestion.srv import AddObject, AddObjectRequest
from task_execution_msgs.srv import GetObjectConstraints


class FindObjectAction(AbstractStep):

    OBJECT_CONSTRAINTS_SERVICE_NAME = "/database/object_constraints"
    SEGMENT_OBJECTS_SERVICE_NAME = "/rail_segmentation/segment_objects"
    PLANNING_SCENE_ADD_SERVICE_NAME = "/grasp_executor/add_object"
    PLANNING_SCENE_CLEAR_SERVICE_NAME = "/grasp_executor/clear_objects"

    def init(self, name):
        self.name = name

        # Objects DB
        self._get_object_constraints_srv = rospy.ServiceProxy(
            FindObjectAction.OBJECT_CONSTRAINTS_SERVICE_NAME,
            GetObjectConstraints
        )

        # The segmentation interface
        self._segment_objects_srv = rospy.ServiceProxy(
            FindObjectAction.SEGMENT_OBJECTS_SERVICE_NAME,
            SegmentObjects
        )

        # The planning scene interface
        self._planning_scene_add_srv = rospy.ServiceProxy(
            FindObjectAction.PLANNING_SCENE_ADD_SERVICE_NAME,
            AddObject
        )
        self._planning_scene_clear_srv = rospy.ServiceProxy(
            FindObjectAction.PLANNING_SCENE_CLEAR_SERVICE_NAME,
            Empty
        )

        # Set a stop flag
        self._stopped = False

        # Wait for a connection to rail_segmentation
        rospy.loginfo("Connecting to rail_segmentation...")
        self._segment_objects_srv.wait_for_service()
        rospy.loginfo("...rail_segmentation connected")

        rospy.loginfo("Connecting to planning_scene...")
        self._planning_scene_add_srv.wait_for_service()
        self._planning_scene_clear_srv.wait_for_service()
        rospy.loginfo("...planning_scene connected")

        rospy.loginfo("Connecting to database services...")
        self._get_object_constraints_srv.wait_for_service()
        rospy.loginfo("...database services connected")

    def run(self, obj):
        # Fetch the object from the database and determing the bounds and
        # location within which the object will be
        obj = obj.split('.', 1)[1]
        rospy.loginfo("Action {}: Inspecting scene for object: {}"
                      .format(self.name, obj))
        self._stopped = False

        # Ask for a segmentation and then identify the object that we want
        segmented_objects = self._segment_objects_srv().segmented_objects
        self.notify_service_called(FindObjectAction.SEGMENT_OBJECTS_SERVICE_NAME)
        yield self.set_running()  # Check on the status of the server
        if self._stopped:
            yield self.set_preempted(
                action=self.name,
                goal=obj,
                srv=self._segment_objects_srv.resolved_name,
                segmented_objects=segmented_objects
            )
            raise StopIteration()

        # Update the planning scene
        self._planning_scene_clear_srv()
        self.notify_service_called(FindObjectAction.PLANNING_SCENE_CLEAR_SERVICE_NAME)
        yield self.set_running()  # Check on the status of the server
        if self._stopped:
            yield self.set_preempted(
                action=self.name,
                goal=obj,
                srv=self._planning_scene_clear_srv.resolved_name,
                segmented_objects=segmented_objects
            )
            raise StopIteration()

        req = AddObjectRequest()
        for idx, segmented_object in enumerate(segmented_objects.objects):
            req.point_clouds.append(segmented_object.point_cloud)
            req.centroids.append(segmented_object.centroid)
            req.indices.append(idx)
        self._planning_scene_add_srv(req)
        self.notify_service_called(FindObjectAction.PLANNING_SCENE_ADD_SERVICE_NAME)
        yield self.set_running()  # Check on the status of the server
        if self._stopped:
            yield self.set_preempted(
                action=self.name,
                goal=obj,
                srv=self._planning_scene_add_srv.resolved_name,
                segmented_objects=segmented_objects
            )
            raise StopIteration()

        # Find the object, based on constraints, among the objects
        found_idx, found_obj = self._find_obj(obj, segmented_objects)
        yield self.set_running()  # Check on the status of the server
        if self._stopped:
            yield self.set_preempted(
                action=self.name,
                goal=obj,
                found_idx=found_idx,
                found_obj=found_obj,
                segmented_objects=segmented_objects
            )
        elif found_idx == -1:
            rospy.logerr("Action {}: FAIL. {} not found among {} objects."
                         .format(self.name, obj, len(segmented_objects.objects)))
            yield self.set_aborted(
                action=self.name,
                goal=obj,
                found_idx=found_idx,
                found_obj=found_obj,
                segmented_objects=segmented_objects
            )
        else:
            yield self.set_succeeded(found_obj=found_obj, found_idx=found_idx)

    def stop(self):
        self._stopped = True

    def _find_obj(self, obj, segmented_objects):
        """
        Find the object with key `obj` in the `segmented_objects` based on the
        details of that object in the database. Our find is rudimentary; it is
        based solely on the expected location and bounds of the object
        """
        obj_constraints = self._get_object_constraints_srv(obj).constraints
        self.notify_service_called(FindObjectAction.OBJECT_CONSTRAINTS_SERVICE_NAME)
        bounds = obj_constraints.bounds if obj_constraints.use_bounds else None
        location = obj_constraints.location if obj_constraints.use_location else None

        found_idx, found_obj = -1, None
        for idx, segmented_object in enumerate(segmented_objects.objects):
            # We assume that the segmentation config is going to take care of
            # putting the point cloud in the appropriate frame for us

            # Check to see if the point cloud is in approximately the expected
            # location
            if location is not None and (
                    segmented_object.center.x < location.xmin or
                    segmented_object.center.x > location.xmax or
                    segmented_object.center.y < location.ymin or
                    segmented_object.center.y > location.ymax or
                    segmented_object.center.z < location.zmin or
                    segmented_object.center.z > location.zmax):
                continue

            # Check to see if the point cloud has approximately the expected
            # dimensions
            if bounds is not None and (
                    segmented_object.bounding_volume.dimensions.x < bounds.xmin or
                    segmented_object.bounding_volume.dimensions.x > bounds.xmax or
                    segmented_object.bounding_volume.dimensions.y < bounds.ymin or
                    segmented_object.bounding_volume.dimensions.y > bounds.ymax or
                    segmented_object.bounding_volume.dimensions.z < bounds.zmin or
                    segmented_object.bounding_volume.dimensions.z > bounds.zmax):
                continue

            # I think we've found the object!
            found_idx, found_obj = idx, segmented_object
            break

        return found_idx, found_obj
