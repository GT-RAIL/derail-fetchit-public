#!/usr/bin/env python
# Monitor the display contacts topic from MoveIt! and fire a blip every time
# MoveIt! thinks the arm is in contact

from __future__ import print_function, division

import rospy

from visualization_msgs.msg import MarkerArray
from task_execution_msgs.msg import MonitorMetadata

from task_monitor.monitoring import AbstractFaultMonitor


# The class definition

class ArmContactMonitor(AbstractFaultMonitor):
    """
    Send a momentary blip every time MoveIt! reports a possible set of contacts
    on the arm. Since the contacts is not reset until the next set of contacts,
    this monitor simply resets after a predefined number of timesteps
    """

    # Monitor specific configs
    ARM_CONTACT_MONITOR_EVENT_NAME = "arm_contact_update"
    ARM_CONTACT_MONITOR_TOPIC = "/move_group/display_contacts"
    ARM_CONTACT_MONITOR_NODES = "/move_group"

    MONITOR_RESET_DURATION = rospy.Duration(5.0)  # Duration after which to reset

    def __init__(self):
        super(ArmContactMonitor, self).__init__()
        self.set_metadata(topics=[ArmContactMonitor.ARM_CONTACT_MONITOR_TOPIC],
                          nodes=[ArmContactMonitor.ARM_CONTACT_MONITOR_NODES])

        # Metadata on the current state from the visualization messages
        self.contacts_present = False
        self._last_contact_time = rospy.Time(0)

        # Subscribers and timers
        self._contacts_sub = rospy.Subscriber(
            ArmContactMonitor.ARM_CONTACT_MONITOR_TOPIC,
            MarkerArray,
            self._on_contact_msg
        )
        self._reset_timer = None

    def _update_contacts_present(self, contacts_present):
        self.contacts_present = contacts_present
        return self.update_trace(
            ArmContactMonitor.ARM_CONTACT_MONITOR_EVENT_NAME,
            self.contacts_present,
            { 'contacts_present': self.contacts_present }
        )

    def _on_contact_msg(self, msg):
        self._last_contact_time = rospy.Time.now()

        # Reset the timer
        if self._reset_timer is not None:
            self._reset_timer.shutdown()
            self._reset_timer = None

        self._reset_timer = rospy.Timer(
            ArmContactMonitor.MONITOR_RESET_DURATION,
            self._reset_contacts,
            oneshot=True
        )

        return self._update_contacts_present(len(msg.markers) > 0)

    def _reset_contacts(self, evt):
        try:
            assert rospy.Time.now() >= self._last_contact_time + ArmContactMonitor.MONITOR_RESET_DURATION, \
                "Timing is off. Last contact @{}; Trying to reset @{}".format(self._last_contact_time, rospy.Time.now())
        except Exception as e:
            rospy.logerr("Error: {}".format(e))
            self._reset_timer = rospy.Timer(
                ArmContactMonitor.MONITOR_RESET_DURATION,
                self._reset_contacts,
                oneshot=True
            )
        else:
            self._reset_timer = None
            return self._update_contacts_present(False)


# When running the monitor in standalone mode
if __name__ == '__main__':
    rospy.init_node('arm_contact_monitor')
    monitor = ArmContactMonitor()
    rospy.spin()
