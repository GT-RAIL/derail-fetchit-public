#!/usr/bin/env python
# Monitor the robot driver and report an error in case the battery levels are
# below critical thresholds

from __future__ import print_function, division

import rospy

from task_execution_msgs.msg import MonitorMetadata
from fetch_driver_msgs.msg import RobotState

from task_monitor.monitoring import AbstractFaultMonitor


# The class definition

class BatteryStateMonitor(AbstractFaultMonitor):
    """
    Monitor the robot state and send out an event alert based on battery levels
    """

    BATTERY_STATE_TOPIC = "/robot_state"
    BATTERY_STATE_MONITOR_EVENT_NAME = "battery_state_update"
    BATTERY_LOW_VOLTAGE = 19.9  # Volts
    BATTERY_LOW_CAPACITY = 6000  # Ah, I think. This is probably >= 10 min of execution time
    BATTERY_CHARGING_MODE = 1  # From comments in the ChargerState.msg

    def __init__(self):
        super(BatteryStateMonitor, self).__init__()
        self.set_metadata(topics=[BatteryStateMonitor.BATTERY_STATE_TOPIC])

        # State variables
        self.battery_state = None

        # Set up the subscriber
        self._robot_state_sub = rospy.Subscriber(
            BatteryStateMonitor.BATTERY_STATE_TOPIC,
            RobotState,
            self._on_robot_state
        )

    def _on_robot_state(self, state_msg):
        self.battery_state = state_msg.charger
        battery_is_low = self.battery_state.charging_mode != BatteryStateMonitor.BATTERY_CHARGING_MODE and (
            self.battery_state.battery_voltage <= BatteryStateMonitor.BATTERY_LOW_VOLTAGE
            or self.battery_state.battery_capacity <= BatteryStateMonitor.BATTERY_LOW_CAPACITY
        )

        return self.update_trace(
            BatteryStateMonitor.BATTERY_STATE_MONITOR_EVENT_NAME,
            battery_is_low,
            { 'battery_state': self.battery_state }
        )


# When running the monitor in standalone mode
if __name__ == '__main__':
    rospy.init_node('robot_state_monitor')
    monitor = BatteryStateMonitor()
    rospy.spin()
