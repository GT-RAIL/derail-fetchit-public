#!/usr/bin/env python
# This class collects information from the various monitors and provides
# fault identification and localization as necessary

from __future__ import print_function, division

import rospy

from task_monitor.execution_tracer import Tracer

from std_srvs.srv import Trigger, TriggerResponse


# The main monitor class

class ExecutionMonitor(object):
    """
    This class is the entrypoint into the combined information available for
    diagnoses
    """

    TRACE_START_SERVICE = '/execution_monitor/start_trace'
    TRACE_STOP_SERVICE = '/execution_monitor/stop_trace'

    def __init__(self):
        # Supplementary execution monitors
        self.tracer = Tracer()

        # Setup the services for the trace
        self._trace_start_service = rospy.Service(ExecutionMonitor.TRACE_START_SERVICE, Trigger, self.start)
        self._trace_stop_service = rospy.Service(ExecutionMonitor.TRACE_STOP_SERVICE, Trigger, self.stop)

    def start(self, req=None):
        # Start the tracer
        self.tracer.start()
        return TriggerResponse(success=True)

    def stop(self, req=None):
        # Stop the tracer
        self.tracer.stop()
        return TriggerResponse(success=True)
