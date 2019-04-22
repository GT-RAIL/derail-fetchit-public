#!/usr/bin/env python
# This file helps determine the recovery actions to take based on the request
# for assistance that was sent in the event of a failure

from __future__ import print_function, division

import copy
import pickle

import rospy

from actionlib_msgs.msg import GoalStatus
from task_execution_msgs.msg import RequestAssistanceResult, ExecuteGoal


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

    def __init__(self, tasks_config):
        self._tasks_config = tasks_config

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

        # Unpickle the context
        context = pickle.loads(assistance_goal.context)

        # A stub recovery in the event of a segmentation failure - simply wait
        if assistance_goal.component == 'segmentation' and 'wait_task' in self._tasks_config:
            execute_goal = ExecuteGoal(name='wait_task')
            resume_hint = RequestAssistanceResult.RESUME_CONTINUE

        return execute_goal, resume_hint
