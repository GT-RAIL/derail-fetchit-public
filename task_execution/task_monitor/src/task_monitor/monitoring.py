#!/usr/bin/env python
# Abstract monitoring classes that each of the individual monitors (belief or
# fault derive from)

from __future__ import print_function, division

import abc
import pickle

import numpy as np

import rospy

from task_execution_msgs.msg import ExecutionEvent, MonitorMetadata, BeliefMetadata

from task_monitor.execution_tracer import Tracer


# The class definitions

class AbstractFaultMonitor(object):
    """All fault monitors should derive from this base class"""

    __metaclass__ = abc.ABCMeta

    def __init__(self):
        self.fault_status = MonitorMetadata.NOMINAL
        self.topics = tuple()
        self.services = tuple()
        self.actions = tuple()
        self.nodes = tuple()

        # The trace publisher
        self._trace = rospy.Publisher(
            Tracer.EXECUTION_TRACE_TOPIC,
            ExecutionEvent,
            queue_size=10
        )

    def set_metadata(self, topics=None, services=None, actions=None, nodes=None):
        """Set the metadata of the monitor. If an arg is None, its value is not
        updated"""
        self.topics = tuple(topics) if topics is not None else self.topics
        self.services = tuple(services) if services is not None else self.services
        self.actions = tuple(actions) if actions is not None else self.actions
        self.nodes = tuple(nodes) if nodes is not None else self.nodes

    def update_trace(self, event_name, fault_status, context=None, force=False):
        """
        Update the trace. If the fault_status is unchanged, do nothing,
        unless the force flag is set.

        fault_status can be a boolean value. If so, then a value of 'True'
        indicates the presence of a fault, while a value of 'False' indicates
        nominal operation.

        Return the message that was sent to the trace
        """
        if isinstance(fault_status, (bool, np.bool,)):
            fault_status = MonitorMetadata.NOMINAL if not fault_status else MonitorMetadata.FAULT

        if fault_status == self.fault_status and not force:
            return None

        # Update the fault_status
        self.fault_status = fault_status
        trace_event = ExecutionEvent(
            stamp=rospy.Time.now(),
            name=event_name,
            type=ExecutionEvent.MONITOR_EVENT,
            monitor_metadata=MonitorMetadata(
                fault_status=self.fault_status,
                context=pickle.dumps(context),
                topics=self.topics,
                services=self.services,
                actions=self.actions,
                nodes=self.nodes
            )
        )
        self._trace.publish(trace_event)
        return trace_event


class AbstractBeliefMonitor(object):
    """All asynchronous belief monitors should derive from this class"""

    __metaclass__ = abc.ABCMeta

    def __init__(self):
        self.beliefs = {}

        # The trace publisher
        self._trace = rospy.Publisher(
            Tracer.EXECUTION_TRACE_TOPIC,
            ExecutionEvent,
            queue_size=10
        )

    def flush_beliefs(self, context=None):
        """Force send ALL beliefs known so far"""
        self.update_beliefs(self.beliefs, context, force=True)

    def update_beliefs(self, beliefs, context=None, force=False):
        """Take in a dictionary of beliefs and update them according to force"""
        # Keep a record of the event messages that were sent
        events_sent = []

        # Update the beliefs as necessary
        for belief, value in beliefs.iteritems():
            value = float(value)
            assert 0 <= value <= 1, "Invalid value for belief, {}: {}".format(belief, value)

            if force or self.beliefs.get(belief, -1) != value:
                trace_event = ExecutionEvent(
                    stamp=rospy.Time.now(),
                    name=belief,
                    type=ExecutionEvent.BELIEF_EVENT,
                    belief_metadata=BeliefMetadata(
                        value=value,
                        context=pickle.dumps(context)
                    )
                )
                self._trace.publish(trace_event)
                events_sent.append(trace_event)

            # Cache the belief
            self.beliefs[belief] = value

        # Finally return the messages that were sent
        return events_sent
