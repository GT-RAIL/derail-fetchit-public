#!/usr/bin/env python
# Gathers beliefs that are published on the trace topic and provides a service
# to query on the robot's belief thus far

from __future__ import print_function, division

import numpy as np

from threading import Lock

import rospy

from std_srvs.srv import Trigger, TriggerResponse
from task_execution_msgs.msg import ExecutionEvent, BeliefKeys
from task_execution_msgs.srv import GetBeliefs, GetBeliefsResponse


# The actual beliefs node

class BeliefsServer(object):
    """
    This server listens on the execution trace topic for updates to beliefs.
    When an update is sent out, it stores the result. A service call can later
    fetch the desired belief value
    """

    EXECUTION_TRACE_TOPIC = '/execution_monitor/trace'

    def __init__(self):
        self._beliefs_lock = Lock()

        # Provide a service to reload; then reload
        self._reload_service = rospy.Service('~reload', Trigger, self.reload)
        self.reload()

        # Create the subscriber
        self._trace_sub = rospy.Subscriber(BeliefsServer.EXECUTION_TRACE_TOPIC, ExecutionEvent, self._on_trace)

        # Create and initialize the service servers
        self._get_beliefs_service = rospy.Service('~get_beliefs', GetBeliefs, self.get_beliefs)

    def start(self):
        # This is a no-op at the moment
        rospy.loginfo("Beliefs node ready...")

    def reload(self, req=None):
        # Reinitialize the dictionary of beliefs
        with self._beliefs_lock:
            self.beliefs = { getattr(BeliefKeys, attr): np.nan for attr in dir(BeliefKeys) if attr.isupper() }

    def _on_trace(self, msg):
        # If this is not a belief event, ignore it
        if msg.type != ExecutionEvent.BELIEF_EVENT:
            return

        # Otherwise, update the known beliefs
        with self._beliefs_lock:
            self.beliefs[getattr(BeliefKeys, msg.name, msg.name)] = msg.belief_metadata.value


    def get_beliefs(self, req):
        # Simply do a dictionary lookup of the beliefs we know about
        beliefs, values = [], []

        with self._beliefs_lock:
            for belief in (req.beliefs or self.beliefs.keys()):
                # Disambiguate the belief
                belief = getattr(BeliefKeys, belief, belief)

                # Then append the belief and the value to the respons
                values.append(self.beliefs[belief])
                beliefs.append(belief)

        return GetBeliefsResponse(beliefs, values)
