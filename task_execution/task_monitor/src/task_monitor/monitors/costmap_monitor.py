#!/usr/bin/env python
# Monitor the costmap and store the percentage of occupied vs. free space

from __future__ import print_function, division

import numpy as np

import rospy

from nav_msgs.msg import OccupancyGrid
from task_execution_msgs.msg import MonitorMetadata

from task_monitor.monitoring import AbstractFaultMonitor


# The class definition

class CostmapMonitor(AbstractFaultMonitor):
    """
    Monitor the local costmap and provide an indication of amount of free space.
    This is a dumb algorithm for now
    """

    COSTMAP_MONITOR_EVENT_NAME = "costmap_update"
    COSTMAP_TOPIC = "/move_base/local_costmap/costmap"
    FREE_PERC_FAULT_THRESHOLD = 0.7  # This is an arbitrary number

    def __init__(self):
        super(CostmapMonitor, self).__init__()
        self.set_metadata(topics=[CostmapMonitor.COSTMAP_TOPIC])

        self.free_space_perc = 1.0

        # Subscribe to the costmap
        self._costmap_sub = rospy.Subscriber(
            CostmapMonitor.COSTMAP_TOPIC,
            OccupancyGrid,
            self._on_costmap
        )

    def _on_costmap(self, costmap_msg):
        self.free_space_perc = 1 - (np.count_nonzero(costmap_msg.data) / len(costmap_msg.data))
        return self.update_trace(
            CostmapMonitor.COSTMAP_MONITOR_EVENT_NAME,
            self.free_space_perc < CostmapMonitor.FREE_PERC_FAULT_THRESHOLD,
            { 'free_space_perc': self.free_space_perc }
        )


# When running the monitor in standalone mode
if __name__ == '__main__':
    rospy.init_node('costmap_monitor')
    monitor = CostmapMonitor()
    rospy.spin()
