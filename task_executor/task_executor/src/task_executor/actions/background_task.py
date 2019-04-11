#!/usr/bin/env python
# Run a background task while the main task loop progresses

import numpy as np

from threading import Thread

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from actionlib_msgs.msg import GoalStatus
from task_execution_msgs.msg import ExecuteAction, ExecuteGoal


# Helpers

def goal_status_from_code(status):
    mapping = {
        GoalStatus.SUCCEEDED: "SUCCEEDED",
        GoalStatus.PREEMPTED: "PREEMPTED",
        GoalStatus.ABORTED: "ABORTED",
    }
    return mapping.get(status, status)


# The class definition

class BackgroundTaskAction(AbstractStep):

    BACKGROUND_TASK_SERVER = "/idle_executor"

    def init(self, name):
        self.name = name

        # Is this background behaviour enabled or is it disabled?
        self.enabled = False

        # The background thread to keep track of the task
        self._background_thread = None

        # The action client
        self._background_client = actionlib.SimpleActionClient(
            BackgroundTaskAction.BACKGROUND_TASK_SERVER,
            ExecuteAction
        )

        # Initialize the action client
        rospy.loginfo("Connecting to the background task executor...")
        self._background_client.wait_for_server()
        rospy.loginfo("...background task executor connected")

    def run(self, task):
        if isinstance(task, (list, tuple,)):
            task = np.random.choice(task)

        rospy.loginfo("Action {}: Starting background task {}".format(self.name, task))

        # Wait for an old thread to complete, if it must
        self.stop()
        while self._background_thread is not None:
            rospy.sleep(0.5)
            yield self.set_running()

        # Create the goal, send it, and update in a new thread
        goal = ExecuteGoal(name=task)
        self._background_client.send_goal(goal)
        self.notify_action_send_goal(BackgroundTaskAction.BACKGROUND_TASK_SERVER, goal)

        self._enabled = True
        self._background_thread = Thread(target=self._check_on_goal)
        self._background_thread.start()

        # Yield a success
        self.set_succeeded()

    def stop(self):
        self._enabled = False

    def _check_on_goal(self):
        # Wait on the background task to complete
        while self._background_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            if not self._enabled or rospy.is_shutdown():
                self._background_client.cancel_goal()
                self.notify_action_cancel(BackgroundTaskAction.BACKGROUND_TASK_SERVER)

            rospy.sleep(0.5)

        # Get the state, result
        status = self._background_client.get_state()
        self._background_client.wait_for_result()
        result = self._background_client.get_result()
        self.notify_action_recv_result(BackgroundTaskAction.BACKGROUND_TASK_SERVER, status, result)

        # Exit cleanly
        rospy.loginfo("Action {}: Finished background task with result {}"
                      .format(self.name, goal_status_from_code(status)))
        self._enabled = False
        self._background_thread = None
