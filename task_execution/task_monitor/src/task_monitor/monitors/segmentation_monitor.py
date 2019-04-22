#!/usr/bin/env python
# Monitor the number of segmented objects. This persists because subsequent\
# calls to segmentation have the potential to reset the fault status

from __future__ import print_function, division

import rospy

from rail_manipulation_msgs.msg import SegmentedObjectList

from task_monitor.monitoring import AbstractFaultMonitor


# The class definition

class SegmentationMonitor(AbstractFaultMonitor):
    """
    Monitor the segmentation topic and send out a fault if the number of objects
    in that topic is 0
    """

    # Monitor specific configs
    SEGMENTATION_MONITOR_EVENT_NAME = "segmentation_update"
    SEGMENTATION_MONITOR_TOPIC = "/rail_segmentation/segmented_objects"

    def __init__(self):
        super(SegmentationMonitor, self).__init__()
        self.set_metadata(topics=[SegmentationMonitor.SEGMENTATION_MONITOR_TOPIC])

        # Setup the subscriber
        self._segmentation_sub = rospy.Subscriber(
            SegmentationMonitor.SEGMENTATION_MONITOR_TOPIC,
            SegmentedObjectList,
            self._on_segmentation
        )

    def _on_segmentation(self, msg):
        return self.update_trace(
            SegmentationMonitor.SEGMENTATION_MONITOR_EVENT_NAME,
            len(msg.objects) == 0,
            { 'num_segmentations': len(msg.objects) }
        )


# When running the monitor in standalone mode
if __name__ == '__main__':
    rospy.init_node('segmentation_monitor')
    monitor = SegmentationMonitor()
    rospy.spin()
