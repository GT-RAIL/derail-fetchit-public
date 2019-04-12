#!/usr/bin/env python
# Send a belief update when the gripper has fully closed

from __future__ import print_function, division

import rospy

from fetch_driver_msgs.msg import GripperState
from task_execution_msgs.msg import BeliefKeys

from task_monitor.monitoring import AbstractBeliefMonitor


# The class definition

class GripperClosedMonitor(AbstractBeliefMonitor):
    """
    Monitor the gripper state and fire a belief update on whether it's fully
    closed or not
    """

    GRIPPER_STATE_TOPIC = "/gripper_state"
    GRIPPER_CLOSED_VALUE = 0.007

    def __init__(self):
        super(GripperClosedMonitor, self).__init__()

        # Setup the subscriber
        self._gripper_state_sub = rospy.Subscriber(
            GripperClosedMonitor.GRIPPER_STATE_TOPIC,
            GripperState,
            self._on_gripper_state
        )

    def _on_gripper_state(self, msg):
        value = msg.joints[0].position <= GripperClosedMonitor.GRIPPER_CLOSED_VALUE
        return self.update_beliefs({ BeliefKeys.GRIPPER_FULLY_CLOSED: value })


# When running the monitor in standalone mode
if __name__ == '__main__':
    rospy.init_node('gripper_closed_monitor')
    monitor = GripperClosedMonitor()
    rospy.spin()
