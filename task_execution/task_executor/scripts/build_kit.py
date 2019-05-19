#!/usr/bin/env python
# Runs a task

from __future__ import print_function, division

import os
import sys
import json
import copy
import time
import pickle
import argparse

import numpy as np

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from task_execution_msgs.msg import ExecuteAction, ExecuteGoal, BeliefKeys
from task_execution_msgs.srv import GetBeliefs
from task_executor.actions import default_actions_dict


class BuildKit:

    BELIEFS_SERVICE_NAME = "/beliefs/get_beliefs"
    BELIEF_KEYS = [x for x in dir(BeliefKeys) if x.isupper()]
    COMPLETE_PICK_PLACE_STATE = ["SMALL_GEAR", "BOLT", "BOLT", "GEARBOX_BOTTOM", "GEARBOX_TOP"]
    # Time expected for pick and place respective object
    PICK_PLACE_TIME = {"SMALL_GEAR": 90,
                       "BOLT": 90,
                       "GEARBOX_BOTTOM": 90,
                       "GEARBOX_TOP": 90}

    def __init__(self):
        rospy.init_node('task_client')

        # Keep track of what objects have been picked and placed for each kit
        self.pick_place_state = []

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

    # ToDo: should have a timed version for picking up schunk if time expires
    def _run(self, task_name, params={}, retry_on_abort=True):
        try_until_succeeded = True
        status = None
        while try_until_succeeded:
            rospy.loginfo("Task {}: START".format(task_name))
            goal = ExecuteGoal(
                name=task_name,
                params=pickle.dumps(params),
            )

            self.task_client.send_goal(goal)
            self.task_client.wait_for_result()
            status = self.task_client.get_state()
            result = self.task_client.get_result()
            variables = (pickle.loads(result.variables) if result.variables != '' else {})
            status = self._goal_status_from_code(status)

            rospy.loginfo("Task {}: {}".format(task_name, status))
            if status == "PREEMPTED":
                exit()
            if status == "ABORTED" and not retry_on_abort:
                break
            if status == "SUCCEEDED":
                break
        return status

    def update_pick_place_state(self):
        resp = self.beliefs_service()
        belief_keys = resp.beliefs
        belief_values = resp.values
        beliefs = {key: value for key, value in zip(belief_keys, belief_values)}
        # rospy.loginfo("BeliefKeys: {}".format(dir(BeliefKeys)))
        # rospy.loginfo("belief keys: {}".format(belief_keys))
        if beliefs["small_gear_in_kit"]:
            if "SMALL_GEAR" not in self.pick_place_state:
                self.pick_place_state.append("SMALL_GEAR")
        if beliefs["gearbox_bottom_in_kit"]:
            if "GEARBOX_BOTTOM" not in self.pick_place_state:
                self.pick_place_state.append("GEARBOX_BOTTOM")
        if beliefs["gearbox_top_in_kit"]:
            if "GEARBOX_TOP" not in self.pick_place_state:
                self.pick_place_state.append("GEARBOX_TOP")
        if beliefs["one_bolt_in_kit"]:
            if self.pick_place_state.count("BOLT") == 0:
                self.pick_place_state.append("BOLT")
        if beliefs["two_bolts_in_kit"]:
            if self.pick_place_state.count("BOLT") == 1:
                self.pick_place_state.append("BOLT")
        rospy.loginfo("pick place state after this update: {}".format(self.pick_place_state))

    def build_kit(self):
        self._run("setup")
        while True:
            self.pick_place_state = []
            self._reset_beliefs()

            self._run("pick_place_kit_on_robot", {"move_location": "waypoints.kit_station",
                                                  "look_location": "gripper_poses.object_look_location"})
            self.fill_kit()
            self._run("pick_place_kit_from_robot", {"move_location": "waypoints.dropoff",
                                                    "bin_location": "BIN_ON_BASE_RIGHT"})


    def fill_kit(self):
        schunk_manip_succeeded = False
        while not schunk_manip_succeeded:
            self._run("pick_insert_gear_in_schunk", {"pick_location": "waypoints.gear_pick_station",
                                                     "pick_look_location": "gripper_poses.object_look_location",
                                                     "schunk_location": "waypoints.schunk_manipulation",
                                                     "schunk_look_location": "gripper_poses.at_schunk_corner"})
            self.simple_pick_place_object(timed=True)
            status = self._run("remove_place_gear_in_kit",
                               {"schunk_location": "waypoints.schunk_manipulation",
                                "schunk_look_location": "gripper_poses.at_schunk_corner"},
                               retry_on_abort=False)
            if status == "SUCCEEDED":
                schunk_manip_succeeded = True

        while len(self._list_diff(self.pick_place_state, BuildKit.COMPLETE_PICK_PLACE_STATE)) > 0:
            self.simple_pick_place_object()

    def simple_pick_place_object(self, timed=False):
        # start timer
        start_time = time.time()  # seconds
        time_left = 120 - (time.time() - start_time)
        while time_left > 0:
            # determine objects that need to be picked
            objects_to_pick = self._list_diff(self.pick_place_state, BuildKit.COMPLETE_PICK_PLACE_STATE)
            rospy.loginfo("Parts left to be picked: {}".format(objects_to_pick))
            if len(objects_to_pick) == 0:
                break
            object_to_pick = objects_to_pick[np.random.choice(len(objects_to_pick))]
            rospy.loginfo("Part to be picked: {}".format(object_to_pick))
            # determine appropriate belief update for the following pick and place action (predict the future)
            belief_update = {}
            move_location = None
            if object_to_pick == "BOLT":
                if self.pick_place_state.count("BOLT") == 0:
                    belief_update["ONE_BOLT_IN_KIT"] = True
                    belief_update["ZERO_BOLTS_IN_KIT"] = False
                elif self.pick_place_state.count("BOLT") == 1:
                    belief_update["TWO_BOLTS_IN_KIT"] = True
                    belief_update["ONE_BOLT_IN_KIT"] = False
                move_location = "waypoints.screw_bin_pick_station"
            else:
                belief_update[object_to_pick + "_IN_KIT"] = True
                if object_to_pick == "SMALL_GEAR":
                    move_location = "waypoints.gear_pick_station"
                elif object_to_pick == "GEARBOX_BOTTOM" or object_to_pick == "GEARBOX_TOP":
                    move_location = "waypoints.gearbox_pick_station"

            self._run("pick_place_object_in_kit", {"object_key": object_to_pick,
                                                                 "move_location": move_location,
                                                                 "look_location": "gripper_poses.object_look_location",
                                                                 "belief_update": belief_update})
            self.update_pick_place_state()

            if timed:
                time_left = 120 - (time.time() - start_time)
                rospy.loginfo("time left (sec): {}".format(time_left))
        return

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

    @staticmethod
    def _goal_status_from_code(status):
        # map from GoalStatus number to string
        mapping = {getattr(GoalStatus, x): x for x in dir(GoalStatus) if x.isupper()}
        return mapping.get(status, status)

    @staticmethod
    def _list_diff(small, large):
        diff = copy.copy(large)
        for item in small:
            diff.remove(item)
        return diff

    @staticmethod
    def _reset_beliefs():
        action = default_actions_dict["update_beliefs"]()
        action.init("update_beliefs")
        rospy.sleep(2.0)
        beliefs = {}
        for belief_key in BuildKit.BELIEF_KEYS:
            beliefs[getattr(BeliefKeys, belief_key)] = False
        status, variables = action(beliefs=beliefs)
        if status == "PREEMPTED" or status == "ABORTED":
            exit()
        return


if __name__ == '__main__':
    bk = BuildKit()
    bk.build_kit()



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
