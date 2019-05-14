#!/usr/bin/env python
# This action server decides the arbitration method given an incoming request

from __future__ import print_function, division

import pickle

from threading import Lock

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from task_execution_msgs.msg import (RequestAssistanceAction,
                                     RequestAssistanceFeedback,
                                     RequestAssistanceResult,
                                     ExecuteAction)

from task_monitor.recovery_strategies import RecoveryStrategies


# The server arbitrates who to send the request to

class TaskMonitorServer(object):
    """
    Given a request for assistance, and some TBD models, the server uses
    the logic in this class to decide whether to request help from local or from
    remote human.
    """

    RECOVERY_ACTION_SERVER = "recovery_executor"
    RECOVERY_TASKS_PARAM = "tasks"  # The full param is /<action_server>/<param>

    # Parameters for the server's behaviour
    CONNECTION_CHECK_DURATION = 0.5  # The seconds to wait before checking for action client connection. Must be > 0.1

    def __init__(self):
        # Create something to hold the action clients that we will be using
        self._recovery_clients = {
            TaskMonitorServer.RECOVERY_ACTION_SERVER: None,
        }
        self._connection_timers = {}
        self._recovery_clients_lock = Lock()

        # Initialize the lookup table of recovery modes
        self._recovery_strategies = RecoveryStrategies(
            rospy.get_param("/{}/{}".format(
                TaskMonitorServer.RECOVERY_ACTION_SERVER,
                TaskMonitorServer.RECOVERY_TASKS_PARAM
            ), {})
        )

        # Instantiate the action server to provide the arbitration
        self._server = actionlib.SimpleActionServer(
            rospy.get_name(),
            RequestAssistanceAction,
            self.execute,
            auto_start=False
        )

    def start(self):
        # Start the connections to the different strategies
        for client_name in self._recovery_clients.keys():
            self._start_connect_to_client(client_name)

        # Start a initialization for the recovery strategies
        self._start_recovery_strategies_init()

        # Start the monitor node itself
        self._server.start()
        rospy.loginfo("Task monitor node ready...")

    def _start_recovery_strategies_init(self):
        def timer_callback(evt):
            self._recovery_strategies.init()

        self._connection_timers["_recovery_strategies_init"] = rospy.Timer(
            rospy.Duration(TaskMonitorServer.CONNECTION_CHECK_DURATION),
            timer_callback,
            oneshot=True
        )

    def _start_connect_to_client(self, client_name):
        rospy.loginfo("Connecting to {}...".format(client_name))

        # Create an action client
        recovery_client = actionlib.SimpleActionClient(client_name, ExecuteAction)

        # Start the periodic checks to see if the client has connected
        self._connection_timers[client_name] = rospy.Timer(
            rospy.Duration(TaskMonitorServer.CONNECTION_CHECK_DURATION),
            self._check_client_connection(client_name, recovery_client),
            oneshot=False
        )

    def _check_client_connection(self, client_name, recovery_client):
        # Create a callback that will be executed for the connection check
        def timer_callback(evt):
            rospy.logdebug("...checking connection to {}...".format(client_name))
            if recovery_client.wait_for_server(rospy.Duration(0.1)):
                # Stop the timer from firing
                self._connection_timers[client_name].shutdown()

                # Set the strategy client
                with self._recovery_clients_lock:
                    self._recovery_clients[client_name] = recovery_client

                rospy.loginfo("...{} connected".format(client_name))

        # Return this callback
        return timer_callback

    def execute(self, goal):
        """Arbitrate an incoming request for assistance"""
        request_received = rospy.Time.now()

        # Pick the strategy
        status = GoalStatus.ABORTED
        result = self._server.get_default_result()
        client_name, recovery_client = None, None
        with self._recovery_clients_lock:
            for name, client in self._recovery_clients.iteritems():
                if client is not None:
                    client_name, recovery_client = name, client
                    break

        # If we do have a valid strategy
        if recovery_client is not None:
            # Unpickle the context
            goal.context = pickle.loads(goal.context)

            # Figure out the execution goal and resume hints
            execute_goal, resume_hint, resume_context = self._recovery_strategies.get_strategy(goal)
            execute_status = (
                GoalStatus.SUCCEEDED
                if execute_goal is None and resume_hint != RequestAssistanceResult.RESUME_NONE
                else status
            )

            if execute_goal is not None:
                # Publish some feedback
                feedback = RequestAssistanceFeedback(strategy=execute_goal.name)
                self._server.publish_feedback(feedback)

                # Send the execute to the recovery client. Preempt if a preempt
                # request has also appeared
                recovery_client.send_goal(execute_goal)
                while not recovery_client.wait_for_result(rospy.Duration(0.5)):
                    if self._server.is_preempt_requested():
                        recovery_client.cancel_goal()

                # Update the result
                execute_status = recovery_client.get_state()
                execute_result = recovery_client.get_result()

                # If the result status is anything other than a success, then
                # resume none. Else, send out the hint that we meant to
                if execute_status != GoalStatus.SUCCEEDED or not execute_result.success:
                    resume_hint = RequestAssistanceResult.RESUME_NONE
                    resume_context = {'resume_hint': resume_hint}

            # Set the result fields
            status = execute_status
            result.resume_hint = resume_hint
            result.context = pickle.dumps(resume_context)
            result.stats.request_complete = rospy.Time.now()
        else:
            # Otherwise, we are aborting the request and sending it back
            result.resume_hint = RequestAssistanceResult.RESUME_NONE
            result.context = goal.context
            result.stats.request_complete = rospy.Time.now()

        # Some extra processing of all results, in case it is needed
        result.stats.request_received = request_received

        # Return based on status
        if status == GoalStatus.SUCCEEDED:
            self._server.set_succeeded(result)
        elif status == GoalStatus.PREEMPTED or self._server.is_preempt_requested():
            self._server.set_preempted(result)
        else:  # Usually, GoalStatus.ABORTED
            self._server.set_aborted(result)

    def stop(self):
        pass
