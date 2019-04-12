#!/usr/bin/env python
# Monitor the global plan and send an error if it happens to pass through an
# obstacle in the local costmap

from __future__ import print_function, division

import numpy as np

import rospy
import tf

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Path
from task_execution_msgs.msg import MonitorMetadata

from task_monitor.monitoring import AbstractFaultMonitor
from task_monitor.utils import costmap_utils


# The class definition

class GlobalPlanMonitor(AbstractFaultMonitor):
    """
    Monitor the global plan and send a message if the plan passes through an
    obstacle in the local costmap
    """

    # Monitor specific configs
    GLOBAL_PLAN_MONITOR_EVENT_NAME = "global_plan_collision_update"
    MONITOR_DURATION = rospy.Duration(1.0)  # Duration at which to check collisions

    # Configuration of the subscribers
    COSTMAP_TOPIC = "/move_base/local_costmap/costmap"
    GLOBAL_PLAN_TOPIC = "/move_base/GlobalPlanner/plan"

    def __init__(self):
        super(GlobalPlanMonitor, self).__init__()
        self.set_metadata(topics=[GlobalPlanMonitor.COSTMAP_TOPIC,
                                  GlobalPlanMonitor.GLOBAL_PLAN_TOPIC])

        self.plan_in_collision = False
        self._latest_plan = None
        self._latest_costmap = None

        # Get the cost translation function and a footprint of 0, 0
        self._footprint = [Point()]
        self._cost_translation_func = costmap_utils.make_cost_translation_func()

        # tf listener
        self._listener = tf.TransformListener()

        # Subscribe to the global plan
        self._global_plan_sub = rospy.Subscriber(
            GlobalPlanMonitor.GLOBAL_PLAN_TOPIC,
            Path,
            self._on_global_plan
        )

        # Subscribe to the costmap
        self._costmap_sub = rospy.Subscriber(
            GlobalPlanMonitor.COSTMAP_TOPIC,
            OccupancyGrid,
            self._on_costmap
        )

        # Start the timer to periodically check for collisions
        self._monitor_timer = rospy.Timer(
            GlobalPlanMonitor.MONITOR_DURATION,
            self._monitor_func,
            oneshot=False
        )

    def _on_global_plan(self, plan_msg):
        self._latest_plan = plan_msg

    def _on_costmap(self, costmap_msg):
        self._latest_costmap = costmap_msg

    def _monitor_func(self, evt):
        # Cache the latest pointers to the costmap and plan to avoid weird race
        # conditions
        costmap, plan = self._latest_costmap, self._latest_plan
        if costmap is None or plan is None:
            return None

        # Iterate through the plan and check for collisions. If found, fire...
        plan_in_collision = False
        for plan_pose in plan.poses:
            # Get the position
            try:
                (trans, rot) = self._listener.lookupTransform(
                    costmap.header.frame_id,
                    plan.header.frame_id,
                    rospy.Time(0)
                )
            except (tf.LookupException, tf.ExtrapolationException):
                return None

            # Try to lookup if the position is in collision. Ignore assertion
            # errors as those simply indicate that we're outside costmap bounds
            try:
                plan_in_collision = costmap_utils.check_collision(
                    plan_pose.pose.position.x + trans[0],  # x
                    plan_pose.pose.position.y + trans[1],  # y
                    self._footprint,
                    costmap,
                    self._cost_translation_func
                )
            except AssertionError as e:
                continue
            except Exception as e:
                rospy.logerr("Error checking collisions: {}".format(e))
                return None

            # Stop if this point is already deemed to be in collision
            if plan_in_collision:
                break

        # Update the value of the plan collision variable and publish to the
        # trace stream
        self.plan_in_collision = plan_in_collision
        return self.update_trace(
            GlobalPlanMonitor.GLOBAL_PLAN_MONITOR_EVENT_NAME,
            self.plan_in_collision,
            { 'plan_in_collision': self.plan_in_collision }
        )


# When running the monitor in standalone mode
if __name__ == '__main__':
    rospy.init_node('global_plan_monitor')
    monitor = GlobalPlanMonitor()
    rospy.spin()
