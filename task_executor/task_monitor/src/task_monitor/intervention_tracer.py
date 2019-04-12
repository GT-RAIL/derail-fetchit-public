#!/usr/bin/env python
# Monitor and process the execution trace

from __future__ import print_function, division

import collections
import numpy as np

import rospy

from task_execution_msgs.msg import (RequestAssistanceActionGoal,
                                     RequestAssistanceResult, InterventionEvent,
                                     InterventionHypothesisMetadata,
                                     InterventionActionMetadata)

from task_monitor.execution_tracer import Tracer as ExecutionTracer


# The tracer class

class Tracer(object):
    """
    Monitors the actions taken during an intervention and logs them to enable
    continual learning of recovery actions. This class functions similarly to
    the Tracer class, which monitors the robot execution trace.
    """

    INTERVENTION_TRACE_TOPIC = '/intervention_monitor/trace'
    MAX_TRACE_LENGTH = 9999

    # Stub event type definition
    TIME_EVENT = ExecutionTracer.TIME_EVENT

    def __init__(self, start_time=None):
        start_time = start_time or rospy.Time.now()

        # Book-keeping variables to keep track of the intervention events
        self.full_trace = collections.deque(maxlen=Tracer.MAX_TRACE_LENGTH)
        self._should_trace = False

        # Initialize the trace
        self.initialize_trace(start_time)

        # The subscriber to track the trace
        self._trace_sub = rospy.Subscriber(
            Tracer.INTERVENTION_TRACE_TOPIC,
            InterventionEvent,
            self.update_trace
        )

    def start(self):
        self._should_trace = True

    def stop(self):
        self._should_trace = False

    def initialize_trace(self, start_time):
        """Initialize the first trace event"""
        # event = InterventionEvent(stamp=start_time)
        # self.full_trace.append(event)
        pass

    def exclude_from_trace(self, msg):
        """Check to see if the message should be excluded from the trace"""
        if not self._should_trace:
            return True

        # Warn if this is an event type that is not recognized
        if msg.type not in [InterventionEvent.START_OR_END_EVENT,
                            InterventionEvent.HYPOTHESIS_EVENT,
                            InterventionEvent.ACTION_EVENT]:
            rospy.logwarn("Unknown event @ {} of type ({})".format(msg.stamp, msg.type))
            return True

        # All is well, include in the trace
        return False

    def update_trace(self, msg):
        if self.exclude_from_trace(msg):
            return

        # Append the full trace
        self.full_trace.append(msg)

        # Then, perform more processing for the specific parts of the trace that
        # we are interested in
