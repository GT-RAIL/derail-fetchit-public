#!/usr/bin/env python
# Base location monitor

from __future__ import print_function, division

import numpy as np

from threading import Lock

import rospy
import tf2_ros

from task_execution_msgs.msg import BeliefKeys
from fetchit_mapping.msg import NavigationActionGoal

from tf2_geometry_msgs import do_transform_pose
from tf.transformations import euler_from_quaternion
from task_monitor.monitoring import AbstractBeliefMonitor


# The class definition

class BaseLocationMonitor(AbstractBeliefMonitor):
    """
    Monitor the location of the base in the map and update the belief of whether
    the robot has reached its last navigation point

    .. todo::

        This monitor should automatically parse out the possible waypoints that
        are defined in the different waypoints files and update the beliefs
        for the desired waypoints automatically. The problem is programmatically
        assigning a defined waypoint to an action in the task
    """

    BASE_FRAME = "base_link"           # Robot position
    DESIRED_COMPARISON_FRAME = "map"   # The frame in which to compare poses
    MONITORING_LOOP_RATE = 10           # Hz
    LOCATION_ERROR = 0.25               # Should be within 25 cm of the goal
    HEADING_ERROR = np.pi / 30          # Should be within 6 deg of the goal

    NAVIGATION_GOAL_TOPIC = "/navigation/goal"

    def __init__(self):
        super(BaseLocationMonitor, self).__init__()

        # Create the tf listener
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # The current pose of the robot
        self._current_location = None

        # The goal sent to the robot
        self._last_goal = None
        self._goal_lock = Lock()

        # Setup the goal subscriber
        self._goal_sub = rospy.Subscriber(
            BaseLocationMonitor.NAVIGATION_GOAL_TOPIC,
            NavigationActionGoal,
            self._on_goal
        )

        # Finally, start the monitor on a timer
        self._monitor_timer = rospy.Timer(
            rospy.Duration(1 / BaseLocationMonitor.MONITORING_LOOP_RATE),
            self._monitor_func,
            oneshot=False
        )

    def _on_goal(self, msg):
        with self._goal_lock:
            # Tansform the incoming goal into a pair of position and RPY in the
            # desired comparison frame
            try:
                transform = self._tf_buffer.lookup_transform(
                    BaseLocationMonitor.DESIRED_COMPARISON_FRAME,
                    msg.goal.goal.header.frame_id,
                    rospy.Time(0)
                )

                tmsg = do_transform_pose(msg.goal.goal, transform)
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                return

            # If the conversion was successful, then save the results
            self._last_goal = {
                'position': np.array([
                    tmsg.pose.position.x,
                    tmsg.pose.position.y,
                    tmsg.pose.position.z,
                ]),
                'heading': np.array(euler_from_quaternion([
                    tmsg.pose.orientation.x,
                    tmsg.pose.orientation.y,
                    tmsg.pose.orientation.z,
                    tmsg.pose.orientation.w
                ])),
            }

    def _monitor_func(self, evt):
        # Get the position of the base
        try:
            transform = self._tf_buffer.lookup_transform(
                BaseLocationMonitor.DESIRED_COMPARISON_FRAME,
                BaseLocationMonitor.BASE_FRAME,
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
            'heading': np.array(euler_from_quaternion([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ])),
        }

        # Get the rotation and translation difference
        with self._goal_lock:
            if self._last_goal is None:
                return

            position_error = np.linalg.norm(self._current_location['position'] - self._last_goal['position'])
            heading_error = np.linalg.norm(self._current_location['heading'] - self._last_goal['heading'])

        # Then update the belief based on whether the position is within the
        # tolerance
        beliefs = {
            BeliefKeys.ROBOT_AT_NAVIGATION_GOAL: (
                position_error <= BaseLocationMonitor.LOCATION_ERROR
                and heading_error <= BaseLocationMonitor.HEADING_ERROR
            )
        }

        # Send the updated values and return the messages that were sent
        return self.update_beliefs(beliefs)


# When running the monitor in standalone mode
if __name__ == '__main__':
    rospy.init_node('base_location_monitor')
    monitor = BaseLocationMonitor()
    rospy.spin()
