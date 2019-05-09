#!/usr/bin/env python
# Semantic location monitor

from __future__ import print_function, division

from math import sin, cos

import numpy as np

import rospy
import tf2_ros

from task_execution_msgs.msg import BeliefKeys
from geometry_msgs.msg import PoseStamped

from tf2_geometry_msgs import do_transform_pose
from tf.transformations import euler_from_quaternion
from task_monitor.monitoring import AbstractBeliefMonitor
from task_execution_msgs.srv import GetWaypoints, GetSemanticLocations


# The class definition

class SemanticLocationMonitor(AbstractBeliefMonitor):
    """
    Automatically parse out the possible semantic locations (waypoints) that are defined in the different waypoints
    files and update the beliefs for the desired semantic locations automatically.
    """

    BASE_FRAME = "base_link"           # Robot position
    DESIRED_COMPARISON_FRAME = "map"   # The frame in which to compare poses
    LOCATION_ERROR = 0.25              # Should be within 25 cm of the goal
    HEADING_ERROR = 0.2         # Should be within 6 deg of the goal
    MONITORING_LOOP_RATE = 10          # Hz

    WAYPOINTS_SERVICE_NAME = "/database/waypoints"
    SEMANTIC_LOCATIONS_SERVICE_NAME = "/database/semantic_locations"

    BELIEF_KEYS = [x for x in dir(BeliefKeys) if x.isupper()]

    def __init__(self):
        super(SemanticLocationMonitor, self).__init__()

        # Create the tf listener
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # The current pose of the robot
        self._current_location = None

        # Get semantic locations and waypoints from data server
        self._get_waypoints_srv = rospy.ServiceProxy(
            SemanticLocationMonitor.WAYPOINTS_SERVICE_NAME,
            GetWaypoints)

        self._get_semantic_locations_srv = rospy.ServiceProxy(
            SemanticLocationMonitor.SEMANTIC_LOCATIONS_SERVICE_NAME,
            GetSemanticLocations
        )

        rospy.loginfo("connecting to database services...")
        self._get_waypoints_srv.wait_for_service()
        rospy.loginfo("...database services connected")

        rospy.loginfo("connecting to database services...")
        self._get_semantic_locations_srv.wait_for_service()
        rospy.loginfo("...database services connected")

        # A dictionary from each semantic location to its' pose in the desired comparison frame
        self._semantic_locations = {}
        self._get_semantic_locations()

        # Finally, start the monitor on a timer
        self._monitor_timer = rospy.Timer(
            rospy.Duration(1.0 / SemanticLocationMonitor.MONITORING_LOOP_RATE),
            self._monitor_func,
            oneshot=False
        )

    def _get_semantic_locations(self):
        locations = self._get_semantic_locations_srv().locations
        for location in locations:
            waypoints = self._get_waypoints_srv(location).waypoints

            # Note: currently, we are only using the last waypoint in the list for each semantic location
            self._semantic_locations[location] = self._compute_pose_from_waypoint(waypoints[-1])

    def _compute_pose_from_waypoint(self, waypoint):
        # Convert the waypoint to a pose
        pose = PoseStamped()
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.orientation.z = sin(waypoint.theta / 2.0)
        pose.pose.orientation.w = cos(waypoint.theta / 2.0)
        pose.header.frame_id = waypoint.frame
        pose.header.stamp = rospy.Time.now()

        # Tansform the pose into a custom pose (pair of position and RPY in the
        # desired comparison frame)
        try:
            transform = self._tf_buffer.lookup_transform(
                SemanticLocationMonitor.DESIRED_COMPARISON_FRAME,
                pose.header.frame_id,
                rospy.Time(0),
                # wait for 100 sec
                rospy.Duration(100)
            )

            pose = do_transform_pose(pose, transform)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            # Note: currently, we are ignoring the exceptions since monitor should not block the main code
            return None

        return pose

    def _monitor_func(self, evt):
        # Get the position of the base
        try:
            transform = self._tf_buffer.lookup_transform(
                SemanticLocationMonitor.DESIRED_COMPARISON_FRAME,
                SemanticLocationMonitor.BASE_FRAME,
                rospy.Time(0)
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            return None

        self._current_location = {
            'position': np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ]),
            #'heading': np.array(euler_from_quaternion([
            'heading': np.array([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]),
        }

        # Update belief based on whether the position is within the tolerance for each semantic location
        beliefs = {}
        for location in self._semantic_locations:
            # rospy.loginfo("{}".format(location))
            pose = self._semantic_locations[location]
            if pose is None:
                continue

            compare_location = {
                'position': np.array([
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                ]),
                # 'heading': np.array(euler_from_quaternion([
                'heading': np.array([
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w
                ]),
            }

            position_error = np.linalg.norm(self._current_location['position'] - compare_location['position'])
            # This cause problems when one angle is near pi and another angle is near -pi
            # heading_error = np.linalg.norm(self._current_location['heading'] - compare_location['heading'])
            # Now compute distance between two quaternions. 0 means same orientation, 1 means 180 degree apart.
            heading_error = 1 - np.inner(self._current_location['heading'], compare_location['heading']) ** 2

            rospy.loginfo("robot at {} position error {} heading error {}".format(location, position_error,
                                                                                      heading_error))
            # Update belief
            belief_key = ("robot_at_" + location).upper()

            if belief_key in SemanticLocationMonitor.BELIEF_KEYS:
                beliefs[getattr(BeliefKeys, belief_key)] = (
                        position_error <= SemanticLocationMonitor.LOCATION_ERROR
                        and heading_error <= SemanticLocationMonitor.HEADING_ERROR
                        )

        # Send the updated values and return the messages that were sent
        return self.update_beliefs(beliefs)


# When running the monitor in standalone mode
if __name__ == '__main__':
    rospy.init_node('semantic_location_monitor')
    monitor = SemanticLocationMonitor()
    rospy.spin()
