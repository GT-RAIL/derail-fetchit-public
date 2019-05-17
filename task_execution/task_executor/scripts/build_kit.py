#!/usr/bin/env python
# Runs a task

from __future__ import print_function, division

import os
import sys
import json
import pickle
import argparse
import time
import numpy as np

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from task_execution_msgs.msg import ExecuteAction, ExecuteGoal
from task_execution_msgs.srv import GetBeliefs


def _goal_status_from_code(status):
    # map from GoalStatus number to string
    mapping = {getattr(GoalStatus, x): x for x in dir(GoalStatus) if x.isupper()}
    return mapping.get(status, status)


def _list_diff(small, large):
    diff = large
    for item in small:
        diff.remove(item)
    return diff


class BuildKit:

    BELIEFS_SERVICE_NAME = "/beliefs/get_beliefs"
    COMPLETE_PICK_PLACE_STATE = ["SMALL_GEAR", "BOLT", "BOLT", "GEARBOX_BOTTOM", "GEARBOX_TOP"]

    def __init__(self):
        rospy.init_node('task_client')

        # Keep track of what objects have been picked and placed for each kit
        self.pick_place_state = []

        # Time expected for pick and place respective object
        self.pick_place_time = {"SMALL_GEAR": 90,
                                "BOLT": 90,
                                "GEARBOX_BOTTOM": 90,
                                "GEARBOX_TOP": 90}

        self.task_client = actionlib.SimpleActionClient("/task_executor", ExecuteAction)

        self.beliefs_service = rospy.ServiceProxy(
            BuildKit.BELIEFS_SERVICE_NAME,
            GetBeliefs
        )

        rospy.loginfo("Connecting to {}...".format("/task_executor"))
        self.task_client.wait_for_server()
        rospy.loginfo("...{} connected".format("/task_executor"))

        rospy.loginfo("Connecting to belief services...")
        self.beliefs_service.wait_for_service()
        rospy.loginfo("...belief services connected")

    def _run(self, task_name, params={}):
        goal = ExecuteGoal(
            name=task_name,
            params=pickle.dumps(params),
        )
        self.task_client.send_goal(goal)
        self.task_client.wait_for_result()
        status = self.task_client.get_state()
        result = self.task_client.get_result()
        variables = (pickle.loads(result.variables) if result.variables != '' else {})
        status = _goal_status_from_code(status)
        if status == "PREEMPTED" or status == "ABORTED":
            exit()
        return status, variables

    def update_pick_place_state(self):
        resp = self.beliefs_service()
        belief_keys = resp.beliefs
        belief_values = resp.values
        beliefs = {key: value for key, value in zip(belief_keys, belief_values)}
        # small gear
        if beliefs["SMALL_GEAR_IN_KIT"]:
            assert "SMALL_GEAR" not in self.pick_place_state
            self.pick_place_state.append("SMALL_GEAR_ON_TABLE")
        if beliefs["GEARBOX_BOTTOM_IN_KIT"]:
            assert "GEARBOX_BOTTOM" not in self.pick_place_state
            self.pick_place_state.append("GEARBOX_BOTTOM")
        if beliefs["GEARBOX_TOP_IN_KIT"]:
            assert "GEARBOX_TOP" not in self.pick_place_state
            self.pick_place_state.append("GEARBOX_TOP")
        if beliefs["ONE_BOLT_IN_KIT"]:
            assert "ONE_BOLT_IN_KIT" not in self.pick_place_state
            self.pick_place_state.append("ONE_BOLT_IN_KIT")
        if beliefs["TWO_BOLTS_IN_KIT"]:
            assert self.pick_place_state.count("ONE_BOLTS_IN_KIT") == 1
            self.pick_place_state.append("ONE_BOLT_IN_KIT")

    def build_kit(self):
        self._run("setup")
        self._run("pick_place_kit_on_robot", {"move_location": "waypoints.kit_station",
                                             "look_location": "gripper_poses.object_look_location"})
        self.fill_kit()
        self._run("pick_place_kit_from_robot", {"move_location": "waypoints.dropoff",
                                               "bin_location": "BIN_ON_BASE_RIGHT"})
        self._run("reposition", {"location": "locations.origin"})

    def fill_kit(self):
        self.pick_place_state = []

        self._run("pick_insert_gear_in_schunk", {"pick_location": "waypoints.gear_pick_station",
                                                "pick_look_location": "gripper_poses.object_look_location",
                                                "schunk_location": "waypoints.schunk_manipulation",
                                                "schunk_look_location": "gripper_poses.at_schunk_corner"})
        self.simple_pick_place_object()
        self._run("remove_place_gear_in_kit", {"schunk_location": "waypoints.schunk_manipulation",
                                              "schunk_look_location": "gripper_poses.at_schunk_corner"})

        while len(_list_diff(self.pick_place_state, BuildKit.COMPLETE_PICK_PLACE_STATE)) > 0:
            status, var = self.simple_pick_place_object()

    def simple_pick_place_object(self):
        # start timer
        start_time = time.time()  # seconds

        # determine objects that need to be picked
        objects_to_pick = _list_diff(self.pick_place_state, BuildKit.COMPLETE_PICK_PLACE_STATE)
        object_to_pick = objects_to_pick[np.random.choice(len(objects_to_pick))]

        # determine appropriate belief update for the following pick and place action (predict the future)
        belief_update = {}
        if object_to_pick == "BOLT":
            if self.pick_place_state.count("BOLT") == 0:
                belief_update["ONE_BOLT_IN_KIT"] = True
                belief_update["ZERO_BOLTS_IN_KIT"] = False
            elif self.pick_place_state.count("BOLT") == 1:
                belief_update["TWO_BOLTS_IN_KIT"] = True
                belief_update["ONE_BOLT_IN_KIT"] = False
        else:
            belief_update[object_to_pick + "_IN_KIT"] = True

        status, var = self._run("pick_place_object_in_kit", {"object_key": object_to_pick,
                                                             "move_location": "waypoints.gear_pick_station",
                                                             "look_location": "gripper_poses.object_look_location",
                                                             "belief_update": belief_update})
        self.update_pick_place_state()
        if status == "ABORTED" or status == "PREEMPTED":
            return status, var
        # ToDo: if pick_place_object failed. it should try again if time permitted.

        time_left = 120 - (time.time() - start_time)
        if time_left > 0:
            status, var = self._run("action", {"duration": time_left})

        return status, var

    def simple_fill_kit(self):
        self._run("pick_insert_gear_in_schunk", {"pick_location": "waypoints.gear_pick_station",
                                                "pick_look_location": "gripper_poses.object_look_location",
                                                "schunk_location": "waypoints.schunk_manipulation",
                                                "schunk_look_location": "gripper_poses.at_schunk_corner"})
        self._run("pick_place_object_in_kit", {"object_key": "SMALL_GEAR",
                                              "move_location": "waypoints.gear_pick_station",
                                              "look_location": "gripper_poses.object_look_location",
                                              "belief_update": {"SMALL_GEAR_ON_TABLE": False,
                                                                "SMALL_GEAR_IN_KIT": True}})
        self._run("action", {"duration": 30.0})
        self._run("remove_place_gear_in_kit", {"schunk_location": "waypoints.schunk_manipulation",
                                              "schunk_look_location": "gripper_poses.at_schunk_corner"})
        self._run("pick_place_object_in_kit", {"object_key": "BOLT",
                                              "move_location": "waypoints.screw_bin_pick_station",
                                              "look_location": "gripper_poses.object_look_location",
                                              "belief_update": {"ZERO_BOLTS_IN_KIT": False,
                                                                "ONE_BOLT_IN_KIT": True}})
        self._run("pick_place_object_in_kit", {"object_key": "BOLT",
                                              "move_location": "waypoints.screw_bin_pick_station",
                                              "look_location": "gripper_poses.object_look_location",
                                              "belief_update": {"ZERO_BOLTS_IN_KIT": False,
                                                                "ONE_BOLT_IN_KIT": True}})
        self._run("pick_place_object_in_kit", {"object_key": "GEARBOX_BOTTOM",
                                              "move_location": "waypoints.screw_bin_pick_station",
                                              "look_location": "gripper_poses.object_look_location",
                                              "belief_update": {"ZERO_BOLTS_IN_KIT": False,
                                                                "ONE_BOLT_IN_KIT": True}})
        self._run("pick_place_object_in_kit", {"object_key": "GEARBOX_TOP",
                                              "move_location": "waypoints.screw_bin_pick_station",
                                              "look_location": "gripper_poses.object_look_location",
                                              "belief_update": {"ZERO_BOLTS_IN_KIT": False,
                                                                "ONE_BOLT_IN_KIT": True}})


if __name__ == '__main__':
    bk = BuildKit()
    bk.build_kit_t()



    # def setup_t(self):
    #     self.run("torso", {"height": 0.4})
    #     self.reset_arm_t({"poses", "joint_poses.ready"})
    #     self.run("detach_objects", {"detach_base": True})
    #     self.run("update_beliefs", {"beliefs": {"ZERO_BOLTS_IN_KIT": True}})
    #
    # def reset_arm_t(self, params):
    #     self.run("load_static_octomap")
    #     self.run("gripper", {"command": "open"})
    #     if "poses" in params:
    #         self.run("arm", {"poses": params["poses"],
    #                          "look_at_gripper": True,
    #                          "max_velocity_scaling": 0.75})
    #
    # def pick_place_kit_on_robot_t(self, params):
    #     self.run("detach_objects", {"detach_arm": True})
    #     self.run("arm", {"poses": "joint_poses.ready",
    #                      "max_velocity_scaling": 0.75})
    #     if "move_location" in params:
    #         self.run("move", {"location": params["move_location"]})
    #     _, var =  self.pick_kit_look_task_t({"look_location": params["look_location"]})
    #     kit_grasp_index = var["kit_grasp_index"]
    #     self.run("move_backward", {"amount": 0.5})
    #     if kit_grasp_index == 0:
    #         self.run("arm", {"poses": "trajectories.kit_pick_0_to_right_place",
    #                          "max_velocity_scaling": 0.75})
    #     else:
    #         self.run("arm", {"poses": "trajectories.kit_pick_1_to_right_place",
    #                          "max_velocity_scaling": 0.75})
    #     _, var = self.run("verify_grasp", {"abort_on_false": True})
    #     grasped = var["grasped"]
    #     self.run("place_kit_base")
    #     _, var = self.run("detect_bins", {"bin_location": "BIN_ON_BASE_RIGHT"})
    #     bin_poses = var["bin_poses"]
    #
    # # ToDo: if a task has defined params and var, check if params is in input
    #
    # def pick_kit_look_task_t(self, params):
    #     grasped = False
    #     while not grasped:
    #         self.pick_kit_task_t(params["look_location"])
    #
    #
    #     return kit_grasp_index


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
