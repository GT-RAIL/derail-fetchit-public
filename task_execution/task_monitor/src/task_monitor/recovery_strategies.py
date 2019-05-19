#!/usr/bin/env python
# This file helps determine the recovery actions to take based on the request
# for assistance that was sent in the event of a failure

from __future__ import print_function, division

import copy
import pickle

import numpy as np

import rospy

from actionlib_msgs.msg import GoalStatus
from task_execution_msgs.msg import (RequestAssistanceResult, ExecuteGoal,
                                     BeliefKeys)
from manipulation_actions.msg import StoreObjectResult, InHandLocalizeResult

from task_executor.actions import get_default_actions


# This class encapsulates the different strategies for recovering from different
# error situations

class RecoveryStrategies(object):
    """
    This class is responsible for determining the recovery task to execute when
    given the request for assistance goal and its corresponding context. The
    primary workhorse of this class is :meth:`get_strategy` which takes in a
    :class:`RequestAssistanceGoal`, and generates an :class:`ExecuteGoal`, a
    :class:`RequestAssistanceResult`, and a ``context`` on how to proceed when
    the recovery execution is complete.
    """

    # Just get all the BeliefKeys
    BELIEF_KEYS = [x for x in dir(BeliefKeys) if x.isupper()]

    # Constant values that can dictate the behaviour of when to apply different
    # recovery behaviours
    MAX_PENULTIMATE_TASK_ABORTS = 7
    MAX_PRIMARY_TASK_ABORTS = 50

    def __init__(self, tasks_config):
        self._tasks_config = tasks_config
        self._actions = get_default_actions()

    def init(self):
        # Initialize the connections of all the actions
        self._actions.init()

    def get_strategy(self, assistance_goal):
        """
        Given an assistance goal, generate an ExecuteGoal that can be used for
        recovery and the corresponding manner of resumption. By default, we
        return a ``None`` ExecuteGoal and a
        :const:`RequestAssistanceResult.NONE`. The former implies that no goal
        should be executed, the latter that the task should be aborted in the
        event of a failure.

        In addition to task specific recoveries that are defined in the various
        ``if-elif-else`` conditions in this method, there are global recovery
        behaviours that apply to prevent infinite loops, for example:

            1. If the number of times the penultimate task in the hierarchy has \
                failed is > :const:`MAX_PENULTIMATE_TASK_ABORTS`, then the \
                recovery is aborted
            2. If the number of times the main task has aborted is > \
                :const:`MAX_PRIMARY_TASK_ABORTS`, then the recovery is aborted

        Args:
            assistance_goal (task_execution_msgs/RequestAssistanceGoal) :
                The request for assistance. The context attribute is unpickled

        Returns:
            (tuple):
                - execute_goal (``task_execution_msgs/ExecuteGoal``) a task \
                    goal to execute if any. If ``None``, assume there is no \
                    such goal
                - resume_hint (``RequestAssistanceResult.resume_hint``) a \
                    constant value indicating how execution should proceed
                - resume_context (dict) more fine grained control of the \
                    intended resume_hint
        """
        execute_goal = None
        resume_hint = RequestAssistanceResult.RESUME_NONE
        resume_context = { 'resume_hint': resume_hint }

        # If our actions are not initialized, then recovery should fail because
        # this is an unknown scenario
        if not self._actions.initialized:
            rospy.logwarn("Recovery: cannot execute because actions are not initialized")
            return execute_goal, resume_hint, resume_context

        # Get the task beliefs. We don't expect it to fail
        _, beliefs = self._actions.get_beliefs(belief_keys=RecoveryStrategies.BELIEF_KEYS)

        # Get the number of times things have failed
        component_names, num_aborts = RecoveryStrategies.get_number_of_component_aborts(assistance_goal.context)

        # NO COMPETITION: Check for the global recovery abort conditions
        # if len(component_names) > 1 and \
        #         num_aborts[-2] > RecoveryStrategies.MAX_PENULTIMATE_TASK_ABORTS:
        #     rospy.loginfo("Recovery: task {} has failed more than {} times".format(
        #         component_names[-2],
        #         RecoveryStrategies.MAX_PENULTIMATE_TASK_ABORTS
        #     ))
        #     return execute_goal, resume_hint, resume_context
        # elif num_aborts[0] > RecoveryStrategies.MAX_PRIMARY_TASK_ABORTS:
        #     rospy.loginfo("Recovery: primary task {} has failed more than {} times".format(
        #         component_names[0],
        #         RecoveryStrategies.MAX_PRIMARY_TASK_ABORTS
        #     ))
        #     return execute_goal, resume_hint, resume_context

        # Then it's a giant lookup table. The first condition in the lookup
        # table is for test tasks. Should NEVER be used during the main task
        if (
            assistance_goal.component == 'loop_body_test'
            or assistance_goal.component == 'reposition_recovery_test'
        ):
            if assistance_goal.component == 'loop_body_test':
                rospy.loginfo("Recovery: simply continue")
                resume_hint = RequestAssistanceResult.RESUME_CONTINUE
                resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)

            elif assistance_goal.component == 'reposition_recovery_test':
                rospy.loginfo("Recovery: reposition the base")
                location = RecoveryStrategies.get_last_goal_location(beliefs)
                assert location is not None, "reposition back to an unknown goal location"
                goal_params = {
                    "origin_move_location": "waypoints.origin_for_" + location,
                    "move_location": "waypoints." + location,
                }
                execute_goal = ExecuteGoal(
                    name="reposition_recovery_task",
                    params=pickle.dumps(goal_params)
                )

        elif (
            assistance_goal.component == 'segment'
            or assistance_goal.component == 'find_grasps'
            or assistance_goal.component == 'retrieve_grasps'
            or assistance_goal.component == 'recognize_object'
        ):
            component_idx = component_names.index(assistance_goal.component)

            if num_aborts[component_idx] <= 3 and not RecoveryStrategies.check_contradictory_beliefs(beliefs):
                rospy.loginfo("Recovery: wait and retry")
                self._actions.wait(duration=0.5)
            else:
                rospy.loginfo("Recovery: reposition, then retry the perception")
                location = RecoveryStrategies.get_last_goal_location(beliefs)
                assert location is not None, "perception task at an unknown goal location"
                goal_params = {
                    "origin_move_location": "waypoints.origin_for_" + location,
                    "move_location": "waypoints." + location,
                }
                execute_goal = ExecuteGoal(
                    name="reposition_recovery_task",
                    params=pickle.dumps(goal_params)
                )

            # If it is any action but segment, retry the whole perception task
            # Alternately, if the base was repositioned, then also retry
            # everything
            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
            resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)
            if (
                'perceive' in component_names
                and (assistance_goal.component != 'segment'
                     or num_aborts[component_idx] > 3
                     or RecoveryStrategies.check_contradictory_beliefs(beliefs))
            ):
                resume_context = RecoveryStrategies.set_task_hint_in_context(
                    resume_context,
                    'perceive',
                    RequestAssistanceResult.RESUME_RETRY
                )

        elif assistance_goal.component == 'detect_bins':
            rospy.loginfo("Recovery: wait before redetect")
            self._actions.wait(duration=0.5)
            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
            resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)
            if 'pick_kit_task' in component_names and num_aborts[-1] >= 3:
                resume_context = RecoveryStrategies.set_task_hint_in_context(
                    resume_context,
                    'pick_kit_task',
                    RequestAssistanceResult.RESUME_RETRY
                )

        elif assistance_goal.component == 'detect_schunk':
            rospy.loginfo("Recovery: wait and try to redetect")
            self._actions.wait(duration=0.5)
            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
            resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)
            if 'detect_schunk_pose_task' in component_names:
                resume_context = RecoveryStrategies.set_task_hint_in_context(
                    resume_context,
                    'detect_schunk_pose_task',
                    RequestAssistanceResult.RESUME_RETRY
                )

        elif (
            assistance_goal.component == 'arm'
            or assistance_goal.component == 'pick'
        ):
            rospy.loginfo("Recovery: wait, then clear octomap")
            self._actions.wait(duration=0.5)

            # Try clearing the octomap
            component_idx = component_names.index(assistance_goal.component)
            self._actions.load_static_octomap()

            # If this is a pick, or an arm step in the pick task then also try
            # moving the arm up from the 2nd failure onwards
            if (
                num_aborts[component_idx] >= 2
                and (len(component_names) > 2 and component_names[-2] == 'pick_task')
            ):
                rospy.loginfo("Recovery: also moving 8cm upwards")
                self._actions.arm_cartesian(linear_amount=[0, 0, 0.08])

            # If this the component has failed at least 5 times, then move the
            # head around to clear the octomap
            if num_aborts[component_idx] >= 5:
                execute_goal = ExecuteGoal(name='clear_octomap_task')

            # Finally, the nuclear option of repositioning, and then restarting
            # everything if the pick action has failed 7 times
            if (
                num_aborts[component_idx] >= 7
                and assistance_goal.component == 'pick'
            ):
                rospy.loginfo("Recovery: reposition, then retry the pick task")
                location = RecoveryStrategies.get_last_goal_location(beliefs)
                assert location is not None, "pick task at an unknown goal location"
                goal_params = {
                    "origin_move_location": "waypoints.origin_for_" + location,
                    "move_location": "waypoints." + location,
                }
                execute_goal = ExecuteGoal(
                    name="reposition_recovery_task",
                    params=pickle.dumps(goal_params)
                )

            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
            resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)

            # If this is a pick, we want to retry segmentation
            if assistance_goal.component == 'pick' and 'perceive_pick' in component_names:
                rospy.loginfo("Recovery: restarting pick with perception")
                resume_context = RecoveryStrategies.set_task_hint_in_context(
                    resume_context,
                    'perceive_pick',
                    RequestAssistanceResult.RESUME_RETRY
                )

        elif assistance_goal.component == 'move':
            rospy.loginfo("Recovery: reposition, then retry move to goal pose")
            component_context = RecoveryStrategies.get_final_component_context(assistance_goal.context)
            self._actions.move_planar(angular_amount=np.pi / 10)
            self._actions.wait(duration=0.5)
            self._actions.move_planar(angular_amount=-1 * np.pi / 10)
            self._actions.wait(duration=0.5)
            self._actions.move_planar(linear_amount=-0.1);
            self._actions.wait(duration=0.5)

            semantic_location_goal = component_context.get('semantic_location')
            if semantic_location_goal is not None:
                goal_params = {
                    "origin_move_location": "waypoints.origin_for_" + semantic_location_goal,
                    "move_location": "waypoints." + semantic_location_goal
                }
                execute_goal = ExecuteGoal(
                    name="reposition_recovery_task",
                    params=pickle.dumps(goal_params)
                )

            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
            resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)

        elif assistance_goal.component == 'reposition':
            rospy.loginfo("Recovery: wiggle back and forth, then retry reposition")
            self._actions.move_planar(angular_amount=np.pi / 10)
            self._actions.wait(duration=0.5)
            self._actions.move_planar(angular_amount=-1 * np.pi / 10)
            self._actions.wait(duration=0.5)
            self._actions.move_planar(linear_amount=-0.1);
            self._actions.wait(duration=0.5)

            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
            resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)

        elif assistance_goal.component == 'move_backward':
            rospy.loginfo("Recovery: move forward a little bit")
            component_context = RecoveryStrategies.get_final_component_context(assistance_goal.context)
            goal_amount = component_context.get('goal', 0.0)
            self._actions.move_backward(amount=-goal_amount/2)
            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
            resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)

        elif (
            assistance_goal.component == 'store_object'
            or assistance_goal.component == 'in_hand_localize'
        ):
            rospy.loginfo("Recovery: wait, then clear octomap")
            self._actions.wait(duration=0.5)
            self._actions.load_static_octomap()
            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
            resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)

            component_context = RecoveryStrategies.get_final_component_context(assistance_goal.context)

            # If this is store object and the result indicates that it exited
            # with a verify grasp failure, then we should redo that object's
            # pick and place
            if (
                assistance_goal.component == 'store_object'
                and 'pick_place_in_kit' in component_names
                and component_context.get('result') is not None
                and component_context['result'].error_code == StoreObjectResult.ABORTED_ON_GRASP_VERIFICATION
            ):
                rospy.loginfo("Recovery: retrying pick-and-place")
                resume_context = RecoveryStrategies.set_task_hint_in_context(
                    resume_context,
                    'pick_place_in_kit',
                    RequestAssistanceResult.RESUME_RETRY
                )

            # If this was a a failure of store when the large gear was in
            # operation, then we need to restart the entire task
            if (
                assistance_goal.component == 'store_object'
                and 'remove_place_gear_in_kit' in component_names
                and 'fill_kit' in component_names
                and component_context.get('result') is not None
                and component_context['result'].error_code == StoreObjectResult.ABORTED_ON_GRASP_VERIFICATION
            ):
                rospy.loginfo("Recovery: have to retry filling the kit")
                resume_context = RecoveryStrategies.set_task_hint_in_context(
                    resume_context,
                    'fill_kit',
                    RequestAssistanceResult.RESUME_RETRY
                )

            # If this is in_hand_localize and the result indicates that we
            # failed with a gear pose check, then we should dropoff the gear and
            # then retry the pick-and-place
            elif (
                assistance_goal.component == 'in_hand_localize'
                and 'pick_insert_gear_in_schunk' in component_names
                and component_context.get('result') is not None
                and component_context['result'].error_code == InHandLocalizeResult.ABORTED_ON_POSE_CHECK
            ):
                rospy.loginfo("Recovery: dropping off the large gear before retrying pick-and-place")
                resume_context = RecoveryStrategies.set_task_hint_in_context(
                    resume_context,
                    'pick_insert_gear_in_schunk',
                    RequestAssistanceResult.RESUME_RETRY
                )
                execute_goal = ExecuteGoal(name="dropoff_unaligned_gear_at_dropoff")

        elif assistance_goal.component == 'pick_kit':
            rospy.loginfo("Recovery: move arm to verify, then retry the pick")
            self._actions.load_static_octomap()
            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
            resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)

        elif assistance_goal.component == 'place_kit_base':
            rospy.loginfo("Recovery: retry the place")
            self._actions.load_static_octomap()
            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
            resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)

        elif assistance_goal.component == 'verify_grasp':
            rospy.loginfo("Recovery: object dropped, retry the pick")
            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
            resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)

            if 'pick_place_in_kit' in component_names:
                resume_context = RecoveryStrategies.set_task_hint_in_context(
                    resume_context,
                    'pick_place_in_kit',
                    RequestAssistanceResult.RESUME_RETRY
                )
            elif 'pick_place_kit_on_robot' in component_names:
                resume_context = RecoveryStrategies.set_task_hint_in_context(
                    resume_context,
                    'pick_place_kit_on_robot',
                    RequestAssistanceResult.RESUME_RETRY
                )
            elif 'pick_insert_gear_in_schunk' in component_names:
                rospy.loginfo("Recovery: pull back and then try again")
                self._actions.move_backward(amount=0.4)
                resume_context = RecoveryStrategies.set_task_hint_in_context(
                    resume_context,
                    'pick_insert_gear_in_schunk',
                    RequestAssistanceResult.RESUME_RETRY
                )
            elif 'pick_from_schunk_task' in component_names:
                rospy.loginfo("Recovery: open gripper and move backwards")
                self._actions.gripper(command="open")
                self._actions.move_backward(amount=0.4)
                resume_context = RecoveryStrategies.set_task_hint_in_context(
                    resume_context,
                    'pick_from_schunk_task',
                    RequestAssistanceResult.RESUME_RETRY
                )
            elif 'remove_place_gear_in_kit' in component_names:
                rospy.loginfo("Recovery: large gear dropped somewhere. Restarting the whole task")
                if 'fill_kit' in component_names:
                    resume_context = RecoveryStrategies.set_task_hint_in_context(
                        resume_context,
                        'fill_kit',
                        RequestAssistanceResult.RESUME_RETRY
                    )
                else:
                    # We are running the pythonized task
                    execute_goal = None
                    resume_hint = RequestAssistanceResult.RESUME_NONE
                    resume_context = { 'resume_hint': resume_hint }

        elif assistance_goal.component == 'approach_schunk':
            rospy.loginfo("Recovery: could not plan to approach pose, clearing octomap and retrying")
            self._actions.load_static_octomap()
            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
            resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)
            if 'arm_approach_schunk_task' in component_names:
                resume_context = RecoveryStrategies.set_task_hint_in_context(
                    resume_context,
                    'arm_approach_schunk_task',
                    RequestAssistanceResult.RESUME_RETRY
                )

            # Nuclear option of repositioning
            if num_aborts[-1] >= 2:
                rospy.loginfo("Recovery: reposition, then retry the approach task")
                location = RecoveryStrategies.get_last_goal_location(beliefs)
                assert location is not None, "approach task at an unknown goal location"
                goal_params = {
                    "origin_move_location": "waypoints.origin_for_" + location,
                    "move_location": "waypoints." + location,
                }
                execute_goal = ExecuteGoal(
                    name="reposition_recovery_task",
                    params=pickle.dumps(goal_params)
                )
                resume_context = RecoveryStrategies.set_task_hint_in_context(
                    resume_context,
                    'insert_in_schunk_task',
                    RequestAssistanceResult.RESUME_PREVIOUS
                )

        elif assistance_goal.component == 'look':
            rospy.loginfo("Recovery: wait and simply retry")
            self._actions.wait(duration=0.5)
            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
            resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)

        elif assistance_goal.component == 'schunk':
            rospy.loginfo("Recovery: wait and then try the schunk again")
            self._actions.wait(duration=5.0)
            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
            resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)

        elif assistance_goal.component == 'schunk_insertion':
            rospy.loginfo("Recovery: move backwards and then try the task again")
            self._actions.move_backward(amount=0.4)
            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
            resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)
            if 'insert_in_schunk_task' in component_names:
                resume_context = RecoveryStrategies.set_task_hint_in_context(
                    resume_context,
                    'insert_in_schunk_task',
                    RequestAssistanceResult.RESUME_RETRY
                )

        elif assistance_goal.component == 'schunk_gripper_pullback':
            rospy.loginfo("Recovery: retry pullback")
            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
            resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)

        elif assistance_goal.component == 'grasp_schunk_gear':
            _, action_result = self._actions.verify_grasp()
            if not action_result.get('grasped'):
                rospy.loginfo("Recovery: No gear in hand. Retry")
                self._actions.move_backward(amount=0.4)
                resume_hint = RequestAssistanceResult.RESUME_CONTINUE
                resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)
                if 'pick_from_schunk_task' in component_names:
                    resume_context = RecoveryStrategies.set_task_hint_in_context(
                        resume_context,
                        'pick_from_schunk_task',
                        RequestAssistanceResult.RESUME_RETRY
                    )
            else:
                rospy.loginfo("Recovery: gear in hand. Move to the next step")
                resume_hint = RequestAssistanceResult.RESUME_CONTINUE
                resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)
                if 'pick_from_schunk_task' in component_names:
                    resume_context = RecoveryStrategies.set_task_hint_in_context(
                        resume_context,
                        'pick_from_schunk_task',
                        RequestAssistanceResult.RESUME_NEXT
                    )

        elif assistance_goal.component == 'retrieve_schunk_gear':
            _, action_result = self._actions.verify_grasp()
            if not action_result.get('grasped'):
                rospy.loginfo("Recovery: No gear in hand. Move back and retry task")
                self._actions.move_backward(amount=0.4)

                # We had the gear and at some point we lost it, have to restart
                if 'fill_kit' in component_names:
                    resume_hint = RequestAssistanceResult.RESUME_CONTINUE
                    resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)
                    resume_context = RecoveryStrategies.set_task_hint_in_context(
                        resume_context,
                        'fill_kit',
                        RequestAssistanceResult.RESUME_RETRY
                    )
                else:
                    execute_goal = None
                    resume_hint = RequestAssistanceResult.RESUME_NONE
                    resume_context = { 'resume_hint': resume_hint }

            else:
                rospy.loginfo("Recovery: gear in hand. Try to move away")
                resume_hint = RequestAssistanceResult.RESUME_CONTINUE
                resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)
                if 'remove_place_gear_in_kit' in component_names:
                    resume_context = RecoveryStrategies.set_task_hint_in_context(
                        resume_context,
                        'remove_place_gear_in_kit',
                        RequestAssistanceResult.RESUME_NEXT
                    )

        # Return the recovery options
        rospy.loginfo("Recovery:\ngoal: {}\nresume_hint: {}\ncontext: {}".format(
            execute_goal if execute_goal is None else execute_goal.name,
            resume_hint,
            resume_context
        ))
        return execute_goal, resume_hint, resume_context

    @staticmethod
    def get_final_component_context(goal_context):
        """
        Get the context of the last component in the context chain
        """
        if len(goal_context.get('context', {})) > 0:
            return RecoveryStrategies.get_final_component_context(goal_context['context'])
        else:
            return goal_context

    @staticmethod
    def get_number_of_component_aborts(goal_context):
        """
        Given the hierarchy of tasks in the goal context, obtain a vector of the
        number of failures in each part of the component of the hierarchy. The
        first index maps to the highest level of the hierarcy and the last
        index maps to the lowest level of the hierarchy.

        Args:
            goal_context (dict) : the goal context

        Returns:
            (tuple):
                - component_names (list) a list of component names from highest \
                    in the task hierarchy to the lowest
                - num_aborts (list) a list of the number of times each \
                    component in component_names aborted
        """
        component_names = []
        num_aborts = []
        sub_context = goal_context

        while sub_context is not None and isinstance(sub_context, dict):
            component_names.append(sub_context.get('task') or sub_context.get('action'))
            num_aborts.append(sub_context.get('num_aborts'))
            sub_context = sub_context.get('context')

        # Return the lists
        return (component_names, num_aborts,)

    @staticmethod
    def create_continue_result_context(goal_context):
        """
        Given the context of a ``task_execution_msgs/RequestAssistanceGoal``
        return a dictionary for a ``task_execution_msgs/RequestAssistanceResult``
        context that indicates :const:`RequestAssistanceResult.RESUME_CONTINUE`

        Args:
            goal_context (dict) : the goal context

        Return:
            (dict) : the result context
        """
        if 'task' in goal_context:
            return {
                'task': goal_context['task'],
                'step_idx': goal_context['step_idx'],
                'resume_hint': RequestAssistanceResult.RESUME_CONTINUE,
                'context': RecoveryStrategies.create_continue_result_context(goal_context['context']),
            }
        else:
            return {}

    @staticmethod
    def set_task_hint_in_context(result_context, task_name, resume_hint):
        """
        Given a result context dictionary, mark the desired task name with the
        desired resume hint

        Args:
            result_context (dict) : the result context, possibly created by
                :meth:`create_continue_result_context`
            task_name (str) : the name of the task
            resume_hint (uint8) : A ``task_execution_msgs/RequestAssistanceResult`` \
                resume_hint constant for the task's resume hint

        Returns:
            (dict) : a result context dictionary with the task set to the \
                desired resume_hint. Note: we do not copy, so the incoming arg \
                might also get affected

        Raises:
            KeyError : if :data:`task_name` is not found in the context
        """

        # Error checking
        if 'task' not in result_context:
            raise KeyError("Expected a result context for tasks. Not found in {}!".format(result_context))

        # If this is not the task we want, then continue on to its context.
        # Otherwise, mark this task as the one we want to update and return
        if result_context['task'] == task_name:
            result_context['resume_hint'] = resume_hint
        else:
            result_context['context'] = RecoveryStrategies.set_task_hint_in_context(result_context['context'], task_name, resume_hint)

        return result_context

    @staticmethod
    def check_contradictory_beliefs(beliefs):
        """
        Given a set of beliefs, check if there is any contradiction.
        Currently only checking the contradiction between task and robot belief about its current location.

        Args:
            beliefs (dict) :

        Returns:
            (bool) : True if there is a contradiction, False otherwise
        """
        robot_beliefs = {}
        task_beliefs = {}
        for belief_key in beliefs:
            # ToDo: lower() may not be necessary
            lower_belief_key = belief_key.lower()
            if "task_at_" in lower_belief_key:
                location = lower_belief_key.replace("task_at_", "")
                task_beliefs[location] = beliefs[belief_key]
            if "robot_at_" in lower_belief_key:
                location = lower_belief_key.replace("robot_at_", "")
                robot_beliefs[location] = beliefs[belief_key]
        for location in set(robot_beliefs.keys()) & set(task_beliefs.keys()):
            if robot_beliefs[location] != task_beliefs[location]:
                return True
        return False

    @staticmethod
    def get_last_goal_location(beliefs):
        """
        Given a set of beliefs, check the last location that the task says the
        robot was trying to get to.
        """
        for belief_key in beliefs:
            # ToDo: lower() may not be necessary
            lower_belief_key = belief_key.lower()
            if "task_at_" in lower_belief_key:
                if beliefs[belief_key] == 1:
                    location = lower_belief_key.replace("task_at_", "")
                    return location
        return None
