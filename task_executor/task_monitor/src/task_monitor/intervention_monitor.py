#!/usr/bin/env python
# This monitors the different changes to the robot during an intervention and
# provides a summary of the actions and results over the course of the
# intervention, as needed

from __future__ import print_function, division

import sys
import collections
import numpy as np

import rospy

from std_srvs.srv import Trigger, TriggerResponse

from task_monitor.intervention_tracer import Tracer

# The main monitor class

class InterventionMonitor(object):
    """
    Similar to the execution monitor
    """

    TRACE_START_SERVICE = '/intervention_monitor/start_trace'
    TRACE_STOP_SERVICE = '/intervention_monitor/stop_trace'

    def __init__(self):
        # The sub-monitors
        self.tracer = Tracer()

        # Setup the services for the trace
        self._trace_start_service = rospy.Service(InterventionMonitor.TRACE_START_SERVICE, Trigger, self.start)
        self._trace_stop_service = rospy.Service(InterventionMonitor.TRACE_STOP_SERVICE, Trigger, self.stop)

    def start(self, req=None):
        self.tracer.start()
        return TriggerResponse(success=True)

    def stop(self, req=None):
        self.tracer.stop()
        return TriggerResponse(success=True)
