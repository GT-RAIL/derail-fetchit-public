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
        return a `None` ExecuteGoal and a RESUME_NONE RequestAssistanceResult.
        The former implies that no goal should be executed, the latter that the
        task should be aborted in the event of a failure.
        """
        execute_goal = None
        resume_hint = RequestAssistanceResult.RESUME_NONE

        # If our actions are not initialized, then recovery should fail because
        # this is an unknown scenario
        if not self._actions.initialized:
            rospy.logwarn("Recovery: cannot execute because actions are not initialized")
            return execute_goal, resume_hint

        # Unpickle the context
        context = pickle.loads(assistance_goal.context)

        # Get the task beliefs
        beliefs = self._actions.get_beliefs(belief_keys=RecoveryStrategies.TASK_BELIEF_KEYS)

        # Then it's a giant lookup table
        if assistance_goal.component == 'segment':
            rospy.loginfo("Recovery: wait before resegment")
            self._actions.wait(duration=0.5)
            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
        elif assistance_goal.component == 'detect_bins':
            rospy.loginfo("Recovery: wait before redetect")
            self._actions.wait(duration=0.5)
            resume_hint = RequestAssistanceResult.RESUME_CONTINUE
        elif (
            assistance_goal.component == 'arm'
            or assistance_goal.component == 'pick'
            or assistance_goal.component == 'in_hand_localize'
        ):
            rospy.loginfo("Recovery: wait, then move head to clear octomap")
            self._actions.wait(duration=0.5)
            execute_goal = ExecuteGoal(name='clear_octomap_task')
            resume_hint = RequestAssistanceResult.RESUME_CONTINUE

        return execute_goal, resume_hint
