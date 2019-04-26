#!/usr/bin/env python
# Monitor the diagnostics topic and report an event if there is a change in a
# diagnostic

from __future__ import print_function, division

import rospy

from task_execution_msgs.msg import MonitorMetadata
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from task_monitor.monitoring import AbstractFaultMonitor


# The class definition

class DiagnosticsMonitor(AbstractFaultMonitor):
    """
    Monitor the /diagnostics topic and return a monitor event in case there is a
    diagnostic that just became faulty, or changed its state from faulty to good
    """

    DIAGNOSTICS_MONITOR_EVENT_NAME = "diagnostics_update"
    DIAGNOSTICS_TOPIC = "/diagnostics"
    SIMULATION_PARAMETER = "/use_sim_time"

    def __init__(self):
        super(DiagnosticsMonitor, self).__init__()
        self.set_metadata(topics=[DiagnosticsMonitor.DIAGNOSTICS_TOPIC])

        self._faulty_diagnostics = {}

        # Need some special logic for simulation here
        self._in_simulation = rospy.get_param(DiagnosticsMonitor.SIMULATION_PARAMETER, False)

        # Create the subscriber to the diagnostics
        self._diagnostics_sub = rospy.Subscriber(
            DiagnosticsMonitor.DIAGNOSTICS_TOPIC,
            DiagnosticArray,
            self._on_diagnostics
        )

    def _on_diagnostics(self, msg):
        # Keep a record of the events that were sent
        events_sent = []

        for diagnostic_status in msg.status:
            # HACK: Update the name if this is in simulation
            if self._in_simulation:
                diagnostic_status.name = diagnostic_status.name.lstrip("robot_driver: ")

            # Figure out if the status of the diagnostic changed
            diagnostic_changed = False
            if diagnostic_status.level == DiagnosticStatus.OK \
                    and diagnostic_status.name in self._faulty_diagnostics:
                del self._faulty_diagnostics[diagnostic_status.name]
                diagnostic_changed = True
            elif diagnostic_status.level != DiagnosticStatus.OK:
                if diagnostic_status.name not in self._faulty_diagnostics:
                    diagnostic_changed = True

                self._faulty_diagnostics[diagnostic_status.name] = diagnostic_status

            # If the status has changed, then force an update on the trace
            if diagnostic_changed:
                events_sent.append(self.update_trace(
                    "{}: {}".format(DiagnosticsMonitor.DIAGNOSTICS_MONITOR_EVENT_NAME, diagnostic_status.name),
                    diagnostic_status.level != DiagnosticStatus.OK,
                    { 'diagnostic': diagnostic_status },
                    force=True
                ))

        # Finally return all the events that were sent
        return events_sent


# When running the monitor in standalone mode
if __name__ == '__main__':
    rospy.init_node('diagnostics_monitor')
    monitor = DiagnosticsMonitor()
    rospy.spin()
