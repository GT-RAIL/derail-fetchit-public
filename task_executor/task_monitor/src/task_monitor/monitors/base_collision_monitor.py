#!/usr/bin/env python
# Monitor the costmap and robot's footprint to check if the robot is in
# collision

from __future__ import print_function, division

import numpy as np

import rospy
import tf

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from task_execution_msgs.msg import MonitorMetadata

from task_monitor.monitoring import AbstractFaultMonitor
from task_monitor.utils import costmap_utils


# The class definition

class BaseCollisionMonitor(AbstractFaultMonitor):
    """
    Monitor the local costmap and the robot's footprint to check if there is a
    collision
    """

    # Monitor specific configs
    BASE_COLLISION_MONITOR_EVENT_NAME = "base_collision_update"
    BASE_COLLISION_MONITOR_NODES = [
        "/move_base",
        "/amcl"
    ]
    MONITOR_DURATION = rospy.Duration(1.0)  # Duration at which to check collisions

    # Configuration for the costmap and other navigation params
    COSTMAP_TOPIC = "/move_base/local_costmap/costmap"
    ROBOT_RADIUS_PARAM = "/move_base/local_costmap/robot_radius"
    ROBOT_FOOTPRINT_NUM_POINTS = 20  # We're going to be naive and simply check the points themselves
    ROBOT_FRAME = "/base_link"

    def __init__(self):
        super(BaseCollisionMonitor, self).__init__()
        self.set_metadata(topics=[BaseCollisionMonitor.COSTMAP_TOPIC],
                          nodes=BaseCollisionMonitor.BASE_COLLISION_MONITOR_NODES)

        self.base_in_collision = False
        self._latest_costmap = None

        # Get the move_base parameter values. Translating C++ -> Python here...
        self._footprint = costmap_utils.make_footprint(
            rospy.get_param(BaseCollisionMonitor.ROBOT_RADIUS_PARAM),
            BaseCollisionMonitor.ROBOT_FOOTPRINT_NUM_POINTS
        )
        self._cost_translation_func = costmap_utils.make_cost_translation_func()

        # tf listener
        self._listener = tf.TransformListener()

        # The subscriber to the costmap
        self._costmap_sub = rospy.Subscriber(
            BaseCollisionMonitor.COSTMAP_TOPIC,
            OccupancyGrid,
            self._on_costmap
        )

        # Start the timer to periodically monitor for collisions
        self._monitor_timer = rospy.Timer(
            BaseCollisionMonitor.MONITOR_DURATION,
            self._monitor_func,
            oneshot=False
        )

    def _on_costmap(self, costmap_msg):
        self._latest_costmap = costmap_msg

    def _monitor_func(self, evt):
        # Cache the pointer to the costmap
        costmap = self._latest_costmap
        if costmap is None:
            return None

        # Get the robot's position
        try:
            (trans, rot) = self._listener.lookupTransform(
                costmap.header.frame_id,
                BaseCollisionMonitor.ROBOT_FRAME,
                rospy.Time(0)
            )
            # Don't need the orientation as we have a circular footprint
            # _, _, theta = tf.transformations.euler_from_quaternion(rot)
        except (tf.LookupException, tf.ExtrapolationException):
            return None

        # Check the collision of each point in the robot's footprint
        try:
            self.base_in_collision = costmap_utils.check_collision(
                trans[0],  # x
                trans[1],  # y
                self._footprint,
                costmap,
                self._cost_translation_func
            )
        except Exception as e:
            rospy.logerr("Error checking collisions: {}".format(e))
            return None

        # Update the trace if necessary
        return self.update_trace(
            BaseCollisionMonitor.BASE_COLLISION_MONITOR_EVENT_NAME,
            self.base_in_collision,
            { 'base_in_collision': self.base_in_collision }
        )


# When running the monitor in standalone mode
if __name__ == '__main__':
    rospy.init_node('base_collision_monitor')
    monitor = BaseCollisionMonitor()
    rospy.spin()
