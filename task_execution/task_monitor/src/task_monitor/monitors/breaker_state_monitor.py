#!/usr/bin/env python
# Monitor the robot driver and report an error in case any of the breakers
# changes state

from __future__ import print_function, division

import numpy as np

import rospy

from task_execution_msgs.msg import MonitorMetadata
from fetch_driver_msgs.msg import RobotState
from power_msgs.msg import BreakerState

from task_monitor.monitoring import AbstractFaultMonitor


# The class definition

class BreakerStateMonitor(AbstractFaultMonitor):
    """
    Monitor the robot state and send out an event alert when there are
    meaningful breaker state changes
    """

    BREAKER_STATE_TOPIC = "/robot_state"
    BREAKER_STATE_MONITOR_EVENT_NAME = "breaker_state_update"

    def __init__(self):
        super(BreakerStateMonitor, self).__init__()
        self.set_metadata(topics=[BreakerStateMonitor.BREAKER_STATE_TOPIC])

        # State variables
        self.breaker_states = None

        # Set up the subscriber
        self._robot_state_sub = rospy.Subscriber(
            BreakerStateMonitor.BREAKER_STATE_TOPIC,
            RobotState,
            self._on_robot_state
        )

    def _on_robot_state(self, state_msg):
        breaker_states = {
            breaker.name: breaker.state for breaker in state_msg.breakers
        }
        trace_event = self.update_trace(
            BreakerStateMonitor.BREAKER_STATE_MONITOR_EVENT_NAME,
            np.any(np.array([state for state in breaker_states.itervalues()]) != BreakerState.STATE_ENABLED),
            { 'breaker_states': breaker_states },
            force=(breaker_states != self.breaker_states)
        )
        self.breaker_states = breaker_states
        return trace_event


# When running the monitor in standalone mode
if __name__ == '__main__':
    rospy.init_node('breaker_state_monitor')
    monitor = BreakerStateMonitor()
    rospy.spin()
