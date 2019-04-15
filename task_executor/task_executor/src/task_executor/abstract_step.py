#!/usr/bin/env python
# The abstract base class that defines an executable step in the task execution
# process

from __future__ import print_function

import abc
import uuid
import pickle

import rospy

from actionlib_msgs.msg import GoalStatus
from task_execution_msgs.msg import (ExecutionEvent, TaskStepMetadata,
                                     MonitorMetadata, BeliefMetadata,
                                     BeliefKeys)


# The class definition

class AbstractStep(object):
    """All steps in a task are derived from this class"""

    __metaclass__ = abc.ABCMeta

    # Definition of GoalStatus codes that indicate "in progress"
    RUNNING_GOAL_STATES = [GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.RECALLING, GoalStatus.PREEMPTING]

    # The execution trace topic
    EXECUTION_TRACE_TOPIC = '/execution_monitor/trace'

    # The set of action, topic, service related events we might care about
    ACTION_SEND_GOAL_EVENT = 'task_action_send_goal'
    ACTION_RECV_RESULT_EVENT = 'task_action_recv_result'
    ACTION_CANCEL_EVENT = 'task_action_cancel'
    SERVICE_CALLED_EVENT = 'task_service_called'
    TOPIC_PUBLISHED_EVENT = 'task_topic_published'
    TOPIC_MESSAGE_EVENT = 'task_topic_message'

    def __init__(self):
        # Set the attributes that all steps have
        self.name = None
        self.uuid = str(uuid.uuid4())
        self._status = GoalStatus.LOST

        # Helpers for updating the trace
        self._trace = rospy.Publisher(AbstractStep.EXECUTION_TRACE_TOPIC, ExecutionEvent, queue_size=10)
        self._last_event = None  # tuple of (event, context,); suppress duplicates

    def __str__(self):
        return "{x.name} ({x.status})".format(x=self)

    def _update_task_trace(self, context):
        # Check to see if this is a trivial update. If so, ignore
        if (self._last_event is not None
                and self._last_event[0].task_step_metadata.status == self.status
                and self._last_event[1] == context):
            return

        # Publish the event
        event = ExecutionEvent(
            stamp=rospy.Time.now(),
            name=self.name,
            type=ExecutionEvent.TASK_STEP_EVENT,
            task_step_metadata=TaskStepMetadata(
                uuid=self.uuid,
                status=self.status,
                context=pickle.dumps(context)
            )
        )
        self._trace.publish(event)
        self._last_event = (event, context,)

    def _update_monitor_trace(self, event_subtype, context, topics=[], services=[], actions=[]):
        # Context must be a dictionary
        context['step_name'] = self.name
        context['step_uuid'] = self.uuid

        # We always publish a monitoring event, because these are manually
        # triggered event notifications in the code
        event = ExecutionEvent(
            stamp=rospy.Time.now(),
            name=event_subtype,  # ACTION_SEND_GOAL_EVENT, ACTION_RECV_RESULT_EVENT, etc.
            type=ExecutionEvent.MONITOR_EVENT,
            monitor_metadata=MonitorMetadata(
                fault_status=MonitorMetadata.NOMINAL,  # We never try to detect faults here
                context=pickle.dumps(context),
                topics=topics,
                services=services,
                actions=actions
            )
        )
        self._trace.publish(event)

    def set_running(self, **context):
        """
        Returns a status denoting that the step is still running
        """
        self._status = GoalStatus.ACTIVE
        self._update_task_trace(context)
        return context

    def set_succeeded(self, **context):
        """
        Returns a status denoting that the step has succeeded
        """
        self._status = GoalStatus.SUCCEEDED
        self._update_task_trace(context)
        return context

    def set_aborted(self, **context):
        """
        Returns a status denoting that the step has failed
        """
        self._status = GoalStatus.ABORTED
        self._update_task_trace(context)
        return context

    def set_preempted(self, **context):
        """
        Returns a status denoting that the step was preempted
        """
        self._status = GoalStatus.PREEMPTED
        self._update_task_trace(context)
        return context

    @property
    def status(self):
        """Current status of this step"""
        return self._status

    def is_running(self):
        """
        Checks to see if the current step is running. Counterpoint to
        `AbstractStep.running()`
        """
        return self._status == GoalStatus.ACTIVE

    def is_succeeded(self):
        """
        Checks to see if the current step is running. Counterpoint to
        `AbstractStep.succeeded()`
        """
        return self._status == GoalStatus.SUCCEEDED

    def is_preempted(self):
        """
        Checks to see if the current step was preempted. Counterpoint to
        `AbstractStep.preempted()`
        """
        return self._status == GoalStatus.PREEMPTED

    def is_aborted(self):
        """
        Checks to see if the current step is running. Counterpoint to
        `AbstractStep.aborted()`
        """
        return not (self.is_running() or self.is_succeeded() or self.is_preempted())

    # def update_beliefs(self, beliefs, context={}):
    #     """Updates the event trace with belief updates. Expects lists/tuples"""
    #     for belief, value in beliefs.iteritems():
    #         value = float(value)
    #         assert 0 <= value <= 1, "Invalid value for belief, {}: {}".format(belief, value)
    #         event = ExecutionEvent(
    #             stamp=rospy.Time.now(),
    #             name=belief,
    #             type=ExecutionEvent.BELIEF_EVENT,
    #             belief_metadata=BeliefMetadata(
    #                 value=value,
    #                 context=pickle.dumps(context)
    #             )
    #         )
    #         self._trace.publish(event)

    def notify_action_send_goal(self, action_server_name, goal):
        """Updates the event trace when a goal is sent to an action server"""
        self._update_monitor_trace(
            AbstractStep.ACTION_SEND_GOAL_EVENT,
            { 'goal': goal },
            actions=[action_server_name]
        )

    def notify_action_recv_result(self, action_server_name, status, result):
        """Updates the event trace when a result is received from an action
        server"""
        self._update_monitor_trace(
            AbstractStep.ACTION_RECV_RESULT_EVENT,
            { 'status': status, 'result': result },
            actions=[action_server_name]
        )

    def notify_action_cancel(self, action_server_name):
        """Updates the event trace when a goal sent to an action server is
        cancelled"""
        self._update_monitor_trace(
            AbstractStep.ACTION_CANCEL_EVENT,
            {},
            actions=[action_server_name]
        )

    def notify_service_called(self, service_name, context={}):
        """Updates the event trace when a service is called"""
        self._update_monitor_trace(
            AbstractStep.SERVICE_CALLED_EVENT,
            context,
            services=[service_name]
        )

    def notify_topic_published(self, topic_name, msg):
        """Updates the event trace when a message is published on a topic"""
        self._update_monitor_trace(
            AbstractStep.TOPIC_PUBLISHED_EVENT,
            { 'msg': msg },
            topics=[topic_name]
        )

    def notify_topic_message(self, topic_name, msg):
        """Updates the event trace when a message is received on a topic"""
        self._update_monitor_trace(
            AbstractStep.TOPIC_MESSAGE_EVENT,
            { 'msg': msg },
            topics=[topic_name]
        )

    @abc.abstractmethod
    def init(self, name, *args, **kwargs):
        """
        Initialize the step.
        """
        raise NotImplementedError()

    @abc.abstractmethod
    def run(self, **params):
        """
        Run the step. This method returns a generator. So don't return;
        instead yield an empty dictionary of return values to keep executing.
        Stop yielding values or raise StopIteration to stop the run.

        Use `running()`, `succeeded()`, `aborted()`, `preempted()` helper
        functions to set status when yielding
        """
        raise NotImplementedError()

    @abc.abstractmethod
    def stop(self):
        """
        Stop the step. How this is handled by the client is up to the client.
        """
        raise NotImplementedError()

    def __call__(self, **params):
        """
        Calls the run generator internally and returns this node's status. This
        is a blocking call.

        Returns:
            status - actionlib_msgs/GoalStatus status_code
            variables - the last yielded dictionary from the run command
        """
        for variables in self.run(**params):
            if rospy.is_shutdown():
                break

        return (self.status, variables,)
