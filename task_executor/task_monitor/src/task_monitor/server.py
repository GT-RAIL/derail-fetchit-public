#!/usr/bin/env python
# This action server decides the arbitration method given an incoming request

from __future__ import print_function, division

import pickle

from threading import Lock

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from task_execution_msgs.msg import RequestAssistanceAction, RequestAssistanceFeedback


# The server arbitrates who to send the request to

class AssistanceArbitrationServer(object):
    """
    Given a request for assistance, and some TBD models, the server uses
    the logic in this class to decide whether to request help from local or from
    remote human.
    """

    # The names of the strategies
    LOCAL_STRATEGY_ACTION_SERVER = "local_strategy"
    REMOTE_STRATEGY_ACTION_SERVER = "remote_strategy"

    # Parameters for the server's behaviour
    CONNECTION_CHECK_DURATION = 0.5  # The seconds to wait before checking for action client connection. Must be > 0.1

    def __init__(self):
        # Create something to hold the action clients that we will be using
        self._strategy_clients = {
            AssistanceArbitrationServer.LOCAL_STRATEGY_ACTION_SERVER: None,
            AssistanceArbitrationServer.REMOTE_STRATEGY_ACTION_SERVER: None,
        }
        self._strategy_connection_timers = {}
        self._strategy_clients_lock = Lock()

        # Instantiate the action server to provide the arbitration
        self._server = actionlib.SimpleActionServer(
            rospy.get_name(),
            RequestAssistanceAction,
            self.execute,
            auto_start=False
        )

    def start(self):
        # Start the connections to the different strategies
        for strategy_name in self._strategy_clients.keys():
            self._start_connect_to_strategy(strategy_name)

        # Start the arbitration node itself
        self._server.start()
        rospy.loginfo("Assistance arbitration node ready...")

    def _start_connect_to_strategy(self, strategy_name):
        rospy.loginfo("Connecting to {}...".format(strategy_name))

        # Create an action client
        strategy_client = actionlib.SimpleActionClient(strategy_name, RequestAssistanceAction)

        # Start the periodic checks to see if the client has connected
        self._strategy_connection_timers[strategy_name] = rospy.Timer(
            rospy.Duration(AssistanceArbitrationServer.CONNECTION_CHECK_DURATION),
            self._check_strategy_connection(strategy_name, strategy_client),
            oneshot=False
        )

    def _check_strategy_connection(self, strategy_name, strategy_client):
        # Create a callback that will be executed for the connection check
        def timer_callback(evt):
            rospy.logdebug("...checking connection to {}...".format(strategy_name))
            if strategy_client.wait_for_server(rospy.Duration(0.1)):
                # Stop the timer from firing
                self._strategy_connection_timers[strategy_name].shutdown()

                # Set the strategy client
                with self._strategy_clients_lock:
                    self._strategy_clients[strategy_name] = strategy_client

                rospy.loginfo("...{} connected".format(strategy_name))

        # Return this callback
        return timer_callback

    def execute(self, goal):
        """Arbitrate an incoming request for assistance"""
        request_received = rospy.Time.now()

        # Pick the strategy
        status = GoalStatus.ABORTED
        result = self._server.get_default_result()
        strategy_name, strategy_client = None, None
        with self._strategy_clients_lock:
            for name, client in self._strategy_clients.iteritems():
                if client is not None:
                    strategy_name, strategy_client = name, client
                    break

        # If we do have a valid strategy
        if strategy_client is not None:
            feedback = RequestAssistanceFeedback(strategy=strategy_name)
            self._server.publish_feedback(feedback)

            # Forward directly to the strategy client. Preempt if a preempt
            # request has also appeared
            strategy_client.send_goal(goal)
            while not strategy_client.wait_for_result(rospy.Duration(0.5)):
                if self._server.is_preempt_requested():
                    strategy_client.cancel_goal()

            # Update the result
            status = strategy_client.get_state()
            result = strategy_client.get_result()
        else:
            # Otherwise, we are aborting the request and sending it back
            result.context = goal.context
            result.stats.request_complete = rospy.Time.now()

        # Some extra processing of all results, in case it is needed
        result.stats.request_received = request_received

        # Return based on status
        if status == GoalStatus.SUCCEEDED:
            self._server.set_succeeded(result)
        elif status == GoalStatus.PREEMPTED:
            self._server.set_preempted(result)
        else:  # Usually, GoalStatus.ABORTED
            self._server.set_aborted(result)

    def stop(self):
        pass
