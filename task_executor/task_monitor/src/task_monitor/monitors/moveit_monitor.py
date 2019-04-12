#!/usr/bin/env python
# Monitor the results topic from MoveIt! and send out diagnostics events if the
# status returned by a MoveIt! execution changes.

from __future__ import print_function, division

import rospy

from task_execution_msgs.msg import MonitorMetadata
from moveit_msgs.msg import MoveGroupActionResult, MoveItErrorCodes
from actionlib_msgs.msg import GoalStatus

from task_monitor.monitoring import AbstractFaultMonitor


# The class definition

class MoveItMonitor(AbstractFaultMonitor):
    """
    Send out an update if there is a problem with the MoveIt Execution based on
    the data in the result topic. We do not try to tie these results to the goal
    messages that were sent.

    There are 4 types of errors that we care about, and an additional catch-all
    condition. If there are successive error messages without a success message
    in between, then all of those messages are sent over. If there is an
    intervening success message, all errors until that message are cleared.
    """

    MOVEIT_MONITOR_EVENT_NAME = "moveit_update: {}"
    MOVEIT_RESULT_TOPIC = "/move_group/result"

    # These events were the most common in the bag files
    MONITORING_EVENTS = {
        MoveItErrorCodes.PLANNING_FAILED: "motion plan failure",
        MoveItErrorCodes.INVALID_MOTION_PLAN: "invalid motion plan",
        MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: "environment changed",
        MoveItErrorCodes.CONTROL_FAILED: "controller failed",
        MoveItErrorCodes.FAILURE: "other failure",  # Catch all condition for other codes
    }

    def __init__(self):
        super(MoveItMonitor, self).__init__()
        self.set_metadata(topics=[MoveItMonitor.MOVEIT_RESULT_TOPIC])

        # The currently active errors
        self._current_errors = {
            event_type: False
            for event_type in MoveItMonitor.MONITORING_EVENTS.keys()
        }

        # Subscribe to messages from MoveIt
        self._result_sub = rospy.Subscriber(
            MoveItMonitor.MOVEIT_RESULT_TOPIC,
            MoveGroupActionResult,
            self._on_result_msg
        )

    def _on_result_msg(self, msg):
        # Check to see if this is a message signalling an error, or signalling
        # a success. We only care about succeeded and aborted statuses at the
        # moment
        events_sent = []

        if msg.status.status == GoalStatus.SUCCEEDED:
            for event_type in self._current_errors.keys():
                # Clear the fault if present. Otherwise, this is a no-op update
                self._current_errors[event_type] = False
                events_sent.append(self.update_trace(
                    MoveItMonitor.MOVEIT_MONITOR_EVENT_NAME.format(MoveItMonitor.MONITORING_EVENTS[event_type]),
                    self._current_errors[event_type],
                    { 'moveit_msg': msg.status.text,
                      'moveit_code': msg.result.error_code.val,
                      'moveit_goal': msg.status.goal_id, },
                    force=True
                ))

        elif msg.status.status == GoalStatus.ABORTED:
            # Only send an update for the error that was expressed
            event_type = msg.result.error_code.val
            if event_type not in self._current_errors.keys():
                event_type = MoveItErrorCodes.FAILURE

            self._current_errors[event_type] = True
            events_sent.append(self.update_trace(
                MoveItMonitor.MOVEIT_MONITOR_EVENT_NAME.format(MoveItMonitor.MONITORING_EVENTS[event_type]),
                self._current_errors[event_type],
                { 'moveit_msg': msg.status.text,
                  'moveit_code': msg.result.error_code.val,
                  'moveit_goal': msg.status.goal_id, },
                force=True
            ))

        # Finally return all the events that were sent
        return events_sent


# When running the monitor in standalone mode
if __name__ == '__main__':
    rospy.init_node('moveit_monitor')
    monitor = MoveItMonitor()
    rospy.spin()
