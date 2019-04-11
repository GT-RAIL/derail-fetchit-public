#!/usr/bin/env python
# Monitor and process the execution trace

from __future__ import print_function, division

import collections
import numpy as np

import rospy

from actionlib_msgs.msg import GoalStatus
from task_execution_msgs.msg import (ExecutionEvent, TaskStepMetadata,
                                     MonitorMetadata, BeliefMetadata,
                                     BeliefKeys)


# Helper functions and classes

class classproperty(property):
    def __get__(self, cls, owner):
        return classmethod(self.fget).__get__(None, owner)()


def get_event_name(event_signature):
    signatures = {
        ExecutionEvent.TASK_STEP_EVENT: 'TASK_STEP',
        ExecutionEvent.ROSGRAPH_EVENT: 'ROSGRAPH',
        ExecutionEvent.MONITOR_EVENT: 'MONITOR',
        ExecutionEvent.BELIEF_EVENT: 'BELIEF',
    }
    return signatures.get(event_signature, event_signature)


def discretize_task_step_status(status):
    statuses = {
        GoalStatus.ACTIVE: 0,
        GoalStatus.SUCCEEDED: 1,
        GoalStatus.ABORTED: -1,
    }
    return statuses.get(status, np.nan)


# The tracer object that collects the trace

class Tracer(object):
    """
    This class monitors the execution trace messages and compiles the data into
    a events trace stream
    """

    EXECUTION_TRACE_TOPIC = '/execution_monitor/trace'
    MAX_TRACE_LENGTH = 9999

    # Stub event type definition
    TIME_EVENT = 255

    # Events to ignore
    EXCLUDE_BELIEF_EVENTS = set([
        BeliefKeys.CUBE_AT_PICKUP_2,
        BeliefKeys.DOOR_2_OPEN,
        BeliefKeys.DOOR_3_OPEN,
    ])
    EXCLUDE_MONITOR_EVENTS = set([
        'diagnostics_update: Charger',
        'diagnostics_update: Gripper IMU Accelerometer',
        'diagnostics_update: Gripper IMU Gyro',
        'diagnostics_update: IMU 1 Accelerometer',
        'diagnostics_update: IMU 1 Gyro',
        'diagnostics_update: Mainboard',
        'diagnostics_update: battery_breaker',
        'diagnostics_update: computer_breaker',
        'diagnostics_update: elbow_flex_mcb',
        'diagnostics_update: forearm_roll_mcb',
        'diagnostics_update: gripper_board_mcb',
        'diagnostics_update: head_pan_mcb',
        'diagnostics_update: head_tilt_mcb',
        'diagnostics_update: joy: Joystick Driver Status',
        'diagnostics_update: joy_node: Joystick Driver Status',
        'diagnostics_update: l_wheel_mcb',
        'diagnostics_update: r_wheel_mcb',
        'diagnostics_update: shoulder_lift_mcb',
        'diagnostics_update: shoulder_pan_mcb',
        'diagnostics_update: sick_tim551_2050001: /base_scan_raw topic status',
        'diagnostics_update: sound_play: Node State',
        'diagnostics_update: supply_breaker',
        'diagnostics_update: torso_lift_mcb',
        'diagnostics_update: upperarm_roll_mcb',
        'diagnostics_update: velodyne_link_nodelet_manager: velodyne_packets topic status',
        'diagnostics_update: wrist_flex_mcb',
        'diagnostics_update: wrist_roll_mcb',
        'task_action_recv_result',
        'task_action_send_goal',
        'task_service_called',
        'task_topic_message',
        'task_topic_published',
    ])
    EXCLUDE_TASK_STEP_EVENTS = set([])

    # Events to include. These are a list because they need to be ordered
    INCLUDE_BELIEF_EVENTS = [
        BeliefKeys.ARM_AT_READY,
        BeliefKeys.ARM_AT_STOW,
        BeliefKeys.ARM_AT_TUCK,
        BeliefKeys.CUBE_AT_DROPOFF,
        BeliefKeys.CUBE_AT_PICKUP_1,
        BeliefKeys.DOOR_1_OPEN,
        BeliefKeys.GRIPPER_FULLY_CLOSED,
        BeliefKeys.GRIPPER_HAS_OBJECT,
        BeliefKeys.TORSO_RAISED,
        BeliefKeys.ROBOT_AT_PICKUP_1,
        BeliefKeys.ROBOT_AT_DOOR_1,
        BeliefKeys.ROBOT_AT_DROPOFF,
    ]
    INCLUDE_MONITOR_EVENTS = [
        'arm_contact_update',
        'base_collision_update',
        'base_stall_update',
        'battery_state_update',
        'breaker_state_update',
        'costmap_update',
        'diagnostics_update: arm_breaker',
        'diagnostics_update: base_breaker',
        'diagnostics_update: gripper_breaker',
        'diagnostics_update: point_cloud_diagnostic: delay',
        'global_plan_collision_update',
        'localization_update',
        'look_direction_update',
        'moveit_update: motion plan failure',
        'moveit_update: invalid motion plan',
        'moveit_update: environment changed',
        'moveit_update: controller failed',
        'moveit_update: other failure',
        'segmentation_update',
        'wifi_update',

        # 'diagnostics_update: Charger',
        # 'diagnostics_update: Gripper IMU Accelerometer',
        # 'diagnostics_update: Gripper IMU Gyro',
        # 'diagnostics_update: IMU 1 Accelerometer',
        # 'diagnostics_update: IMU 1 Gyro',
        # 'diagnostics_update: Mainboard',
        # 'diagnostics_update: battery_breaker',
        # 'diagnostics_update: computer_breaker',
        # 'diagnostics_update: elbow_flex_mcb',
        # 'diagnostics_update: forearm_roll_mcb',
        # 'diagnostics_update: gripper_board_mcb',
        # 'diagnostics_update: head_pan_mcb',
        # 'diagnostics_update: head_tilt_mcb',
        # 'diagnostics_update: joy: Joystick Driver Status',
        # 'diagnostics_update: joy_node: Joystick Driver Status',
        # 'diagnostics_update: l_wheel_mcb',
        # 'diagnostics_update: r_wheel_mcb',
        # 'diagnostics_update: shoulder_lift_mcb',
        # 'diagnostics_update: shoulder_pan_mcb',
        # 'diagnostics_update: sick_tim551_2050001: /base_scan_raw topic status',
        # 'diagnostics_update: sound_play: Node State',
        # 'diagnostics_update: supply_breaker',
        # 'diagnostics_update: torso_lift_mcb',
        # 'diagnostics_update: upperarm_roll_mcb',
        # 'diagnostics_update: velodyne_link_nodelet_manager: velodyne_packets topic status',
        # 'diagnostics_update: wrist_flex_mcb',
        # 'diagnostics_update: wrist_roll_mcb',
    ]
    INCLUDE_TASK_STEP_EVENTS = [
        'approach',
        'arm',
        'arm_place',
        'beep',
        'check_obstacle_in_front',
        'choose_and_traverse_door',
        'choose_first_true_belief',
        'depart',
        'detach_objects',
        'easy',
        'easy_bracket',
        'find_grasps',
        'find_object',
        'gripper',
        'hard',
        'hard_bracket',
        'look',
        'look_at_gripper_arm',
        'look_look_at_gripper',
        'move',
        'perceive',
        'perceive_and_pick',
        'pick',
        'pick_task',
        'place',
        'place_task',
        'reset_arm',
        'setup',
        'speak',
        'torso',
        'traverse_doorway',
        'traverse_doorways',
        'update_beliefs',
        'wait',
    ]

    # This is a vector, used to index into rows
    _trace_types = None
    _trace_types_idx = None

    def __init__(self, start_time=None):
        start_time = start_time or rospy.Time.now()

        # Book-keeping variables to keep track of the trace state
        self.full_trace = collections.deque(maxlen=Tracer.MAX_TRACE_LENGTH)
        self._trace = np.ones((len(Tracer.trace_types), Tracer.MAX_TRACE_LENGTH,), dtype=np.float) * np.nan
        self._should_trace = False  # Variable that determines whether to trace

        # Book-keeping variables to help with the trace parsing
        self._tasks_to_reset = set()

        # Initialize the trace
        self.initialize_trace(start_time)

        # Setup the subscriber to track the trace
        self._trace_sub = rospy.Subscriber(
            Tracer.EXECUTION_TRACE_TOPIC,
            ExecutionEvent,
            self.update_trace
        )

    @classproperty
    def trace_types(cls):
        if cls._trace_types is None:
            cls._trace_types = (
                [(Tracer.TIME_EVENT, 'time')]
                + [(ExecutionEvent.BELIEF_EVENT, name) for name in cls.INCLUDE_BELIEF_EVENTS]
                + [(ExecutionEvent.MONITOR_EVENT, name) for name in cls.INCLUDE_MONITOR_EVENTS]
                + [(ExecutionEvent.TASK_STEP_EVENT, name) for name in cls.INCLUDE_TASK_STEP_EVENTS]
            )

        return cls._trace_types

    @classproperty
    def trace_types_idx(cls):
        if cls._trace_types_idx is None:
            cls._trace_types_idx = { x: i for i, x in enumerate(Tracer.trace_types) }
        return cls._trace_types_idx

    @staticmethod
    def trace_idx_by_type(trace_type):
        if trace_type == ExecutionEvent.BELIEF_EVENT:
            trace_names = Tracer.INCLUDE_BELIEF_EVENTS
        elif trace_type == ExecutionEvent.MONITOR_EVENT:
            trace_names = Tracer.INCLUDE_MONITOR_EVENTS
        elif trace_type == ExecutionEvent.TASK_STEP_EVENT:
            trace_names = Tracer.INCLUDE_TASK_STEP_EVENTS
        else:
            raise ValueError("Unknown trace type: {}".format(trace_type))

        return [Tracer.trace_types_idx[(trace_type, n,)] for n in trace_names]

    @property
    def num_events(self):
        return len(self.full_trace)

    @property
    def last_event(self):
        return self.full_trace[-1]

    @property
    def trace(self):
        return self._trace[:, :self.num_events]

    def start(self):
        self._should_trace = True

    def stop(self):
        self._should_trace = False

    def initialize_trace(self, start_time):
        """Initialize the first trace event"""
        event = ExecutionEvent(stamp=start_time)
        self.full_trace.append(event)

        self._trace[0, 0] = start_time.to_time()
        for idx, trace_spec in enumerate(Tracer.trace_types):
            if trace_spec[0] == ExecutionEvent.MONITOR_EVENT:
                self._trace[idx, 0] = MonitorMetadata.NOMINAL

    def exclude_from_trace(self, msg):
        """Check to see if the msg should be excluded from the trace"""
        # We want to exclude if the tracer hasn't been started
        if not self._should_trace:
            return True

        # We want to ignore ROSGRAPH events
        if msg.type == ExecutionEvent.ROSGRAPH_EVENT:
            return True

        # Make sure to exclude the subevents that we don't care about
        if msg.name in Tracer.EXCLUDE_BELIEF_EVENTS and msg.type == ExecutionEvent.BELIEF_EVENT:
            return True

        if msg.name in Tracer.EXCLUDE_MONITOR_EVENTS and msg.type == ExecutionEvent.MONITOR_EVENT:
            return True

        if msg.name in Tracer.EXCLUDE_TASK_STEP_EVENTS and msg.type == ExecutionEvent.TASK_STEP_EVENT:
            return True

        # Warn if there is an event that isn't a known trace type
        if (msg.type, msg.name) not in Tracer.trace_types:
            rospy.logwarn("Unknown event {} ({})"
                          .format(msg.name, get_event_name(msg.type)))
            return True

        # All is well, include in the trace
        return False

    def update_trace(self, msg):
        """As messages come in, update the trace"""
        if self.exclude_from_trace(msg):
            return

        # Append to the full trace
        num_events = self.num_events  # Keep track of the old num_events
        self.full_trace.append(msg)

        # Copy over the previous time-step's trace. Also recycle if the trace
        # is too long
        if num_events == Tracer.MAX_TRACE_LENGTH:
            self._trace[:, :Tracer.MAX_TRACE_LENGTH-1] = self._trace[:, 1:]

        self._trace[:, num_events] = self._trace[:, num_events-1]
        current_evt = self._trace[:, num_events]
        current_evt[0] = msg.stamp.to_time()

        # Update all the tasks that have completed
        for task_spec in self._tasks_to_reset:
            current_evt[Tracer.trace_types_idx[task_spec]] = np.nan
        self._tasks_to_reset = set()

        # Update the task step according to the incoming data
        if msg.type == ExecutionEvent.BELIEF_EVENT:
            current_evt[Tracer.trace_types_idx[(msg.type, msg.name,)]] = \
                msg.belief_metadata.value
        elif msg.type == ExecutionEvent.MONITOR_EVENT:
            current_evt[Tracer.trace_types_idx[(msg.type, msg.name,)]] = \
                msg.monitor_metadata.fault_status
        elif msg.type == ExecutionEvent.TASK_STEP_EVENT:
            # Add the task to the list of tasks completed during this iteration
            if msg.task_step_metadata.status in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED]:
                self._tasks_to_reset.add((msg.type, msg.name,))

            # Get the discretized status: -1, 0, 1
            status = discretize_task_step_status(msg.task_step_metadata.status)
            current_evt[Tracer.trace_types_idx[(msg.type, msg.name,)]] = status
        else:
            raise Exception("Unrecognized event {} of type {}"
                            .format(msg.name, msg.type))
