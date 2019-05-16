#!/usr/bin/env python
# Runs a task

from __future__ import print_function, division

# import os
# import sys
# import json
import pickle
# import argparse

# from ruamel.yaml import YAML

import rospy
# import rospkg
import actionlib

# from actionlib_msgs.msg import GoalStatus
from task_execution_msgs.msg import ExecuteAction, ExecuteGoal

# from task_executor.tasks import Task


# yaml = YAML(typ='safe')


def setup():
    rospy.init_node('task_client')
    client = actionlib.SimpleActionClient("/task_executor", ExecuteAction)
    rospy.loginfo("Connecting to {}...".format("/task_executor"))
    client.wait_for_server()
    rospy.loginfo("...{} connected".format("/task_executor"))
    return client


def build_kit(client):
    # setup
    run(client, "setup")
    run(client, "pick_place_kit_on_robot", {"move_location": "waypoints.kit_station",
                                            "look_location": "gripper_poses.object_look_location"})
    run(client, "fill_kit")
    run(client, "pick_place_kit_from_robot", {"move_location": "waypoints.dropoff",
                                              "bin_location": "BIN_ON_BASE_RIGHT"})
    run(client, "reposition", {"location": "locations.origin"})


def run(client, task_name, params):
    goal = ExecuteGoal(
        name=task_name,
        params=pickle.dumps(params),
    )
    client.send_goal(goal)
    client.wait_for_result()
    status = client.get_state()
    result = client.get_result()
    return status, result


if __name__ == '__main__':
    client = setup()
    build_kit(client)

# def run_action(action, params={}, )
#
#
# def _goal_status_from_code(status):
#     mapping = { getattr(GoalStatus, x): x for x in dir(GoalStatus) if x.isupper() }
#     return mapping.get(status, status)
#
#
# def _get_arg_parser():
#     # First load the list of available tasks
#     default_tasks_file = os.path.join(rospkg.RosPack().get_path('task_executor'),
#                                       'config/tasks.yaml')
#     with open(default_tasks_file, 'r') as fd:
#         tasks = yaml.load(fd)['tasks']
#
#     # Then create the parser
#     parser = argparse.ArgumentParser()
#     parser.add_argument('task_name', choices=tasks.keys(),
#                         help="Name of the task to run")
#     parser.add_argument('--server_name', default="/task_executor")
#     parser.add_argument('--no-recoveries', action='store_true',
#                         help="Don't run recovery in the case of failures")
#     parser.add_argument('--params', default='{}',
#                         help="params to the task provided as a JSON string")
#     return parser
#
#
# def main():
#     parser = _get_arg_parser()
#     args = parser.parse_args(rospy.myargv(sys.argv)[1:])
#
#     rospy.init_node('task_client')
#     client = actionlib.SimpleActionClient(args.server_name, ExecuteAction)
#     rospy.loginfo("Connecting to {}...".format(args.server_name))
#     client.wait_for_server()
#     rospy.loginfo("...{} connected".format(args.server_name))
#
#     # Read the params
#     def convert_params(d):
#         new_d = {}
#         for k, v in d.iteritems():
#             if isinstance(k, unicode):
#                 k = str(k)
#             new_d[k] = v
#
#             if isinstance(v, unicode):
#                 new_d[k] = str(v)
#             elif isinstance(v, dict):
#                 new_d[k] = convert_params(v)
#
#         return new_d
#
#     params = json.loads(args.params)
#     params = convert_params(params)
#
#     goal = ExecuteGoal(
#         name=args.task_name,
#         params=pickle.dumps(params),
#         no_recoveries=args.no_recoveries
#     )
#     client.send_goal(goal)
#     client.wait_for_result()
#
#     status = client.get_state()
#     result = client.get_result()
#     variables = (pickle.loads(result.variables) if result.variables != '' else {})
#     rospy.loginfo("Result: {}. Variables:\n{}".format(
#         _goal_status_from_code(status),
#         Task.pprint_variables(variables)
#     ))
