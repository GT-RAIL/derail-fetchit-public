#!/usr/bin/env python
# This file helps determine the recovery actions to take based on the request
# for assistance that was sent in the event of a failure

from __future__ import print_function, division

import copy
import pickle

import rospy

from actionlib_msgs.msg import GoalStatus
from task_execution_msgs.msg import (RequestAssistanceResult, ExecuteGoal,
                                     BeliefKeys)

from task_executor.actions import get_default_actions


# This class encapsulates the different strategies for recovering from different
# error situations

class RecoveryStrategies(object):
    """
    This class is responsible for determining the recovery task to execute when
    given the request for assistance goal and its corresponding context. The
    primary workhorse of this class is the `get_strategy` function which takes
    in a RequestAssistanceGoal, and generates an ExecuteGoal for recovery as
    well as a RequestAssistanceResult for how to proceed when the execution is
    complete.
    """

    # The belief keys that correspond to the semantic state of the task
    TASK_BELIEF_KEYS = [
        BeliefKeys.LARGE_GEAR_ON_TABLE,
        BeliefKeys.LARGE_GEAR_IN_SCHUNK,
        BeliefKeys.LARGE_GEAR_IN_KIT,

        BeliefKeys.SMALL_GEAR_ON_TABLE,
        BeliefKeys.SMALL_GEAR_IN_KIT,

        BeliefKeys.ZERO_BOLTS_IN_KIT,
        BeliefKeys.ONE_BOLT_IN_KIT,
        BeliefKeys.TWO_BOLTS_IN_KIT,

        BeliefKeys.KIT_ON_TABLE,
        BeliefKeys.KIT_ON_ROBOT,
        BeliefKeys.KIT_COMPLETE,

        BeliefKeys.SCHUNK_IS_MACHINING,
    ]

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
            return execute_goal, resume_hint

        # Get the task beliefs
        beliefs = self._actions.get_beliefs(belief_keys=RecoveryStrategies.TASK_BELIEF_KEYS)

        # Then it's a giant lookup table
        if assistance_goal.component == 'segment':
            rospy.loginfo("Recovery: wait before resegment")
            self._actions.wait(duration=0.5)
            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
            resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)
        elif assistance_goal.component == 'detect_bins':
            rospy.loginfo("Recovery: wait before redetect")
            self._actions.wait(duration=0.5)
            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
            resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)
        elif (
            assistance_goal.component == 'arm'
            or assistance_goal.component == 'pick'
            or assistance_goal.component == 'in_hand_localize'
        ):
            rospy.loginfo("Recovery: wait, then move head to clear octomap")
            self._actions.wait(duration=0.5)
            execute_goal = ExecuteGoal(name='clear_octomap_task')
            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
            resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)

            # If this is a pick, we want to retry segmentation
            if assistance_goal.component == 'pick':
                rospy.loginfo("Recovery: restarting pick with perception")
                resume_context = RecoveryStrategies.set_task_hint_in_context(
                    resume_context,
                    'perceive_pick',
                    RequestAssistanceResult.RESUME_RETRY
                )
        elif assistance_goal.component == 'find_grasps':
            rospy.loginfo("Recovery: wait, then retry the perception")
            self._actions.wait(duration=0.5)
            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
            resume_context = RecoveryStrategies.create_continue_result_context(assistance_goal.context)
            resume_context = RecoveryStrategies.set_task_hint_in_context(
                resume_context,
                'perceive',
                RequestAssistanceResult.RESUME_RETRY
            )

        # Return the recovery options
        rospy.loginfo("Recovery:\ngoal: {}\nresume_hint: {}\ncontext: {}".format(
            execute_goal.name, resume_hint, resume_context
        ))
        return execute_goal, resume_hint, resume_context

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
