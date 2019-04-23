#!/usr/bin/env python
# Runs a task

from __future__ import print_function, division

import sys
import argparse

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from task_execution_msgs.msg import ExecuteAction, ExecuteGoal


def _goal_status_from_code(status):
    mapping = {
        GoalStatus.SUCCEEDED: "SUCCEEDED",
        GoalStatus.PREEMPTED: "PREEMPTED",
        GoalStatus.ABORTED: "ABORTED",
    }
    return mapping.get(status, status)


def _get_arg_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('task_name', help="Name of the task to run")
    parser.add_argument('--server_name', default="/task_executor")
    return parser


def main():
    parser = _get_arg_parser()
    args = parser.parse_args()

    rospy.init_node('task_client')
    client = actionlib.SimpleActionClient(args.server_name, ExecuteAction)
    rospy.loginfo("Connecting to {}...".format(args.server_name))
    client.wait_for_server()
    rospy.loginfo("...{} connected".format(args.server_name))

    goal = ExecuteGoal(name=args.task_name)
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Result: {}".format(
        _goal_status_from_code(client.get_state())
    ))


if __name__ == '__main__':
    main()
