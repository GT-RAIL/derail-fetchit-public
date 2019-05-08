#!/usr/bin/env python
# The hat action in the task plan

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from actionlib_msgs.msg import GoalStatus
from data_recorder.msg import PlaybackAction, PlaybackGoal


# The class definition

class HatAction(AbstractStep):

    PLAYBACK_ACTION_SERVER = "/playback_primitive"
    HAT_COMMANDS = ["hat_reach", "hat_maneuver", "hat_retract"]

    def init(self, name):
        self.name = name
        self._hat_client = actionlib.SimpleActionClient(
            HatAction.PLAYBACK_ACTION_SERVER,
            PlaybackAction
        )

        rospy.loginfo("Connecting to hat executor...")
        self._hat_client.wait_for_server()
        rospy.loginfo("...hat executor connected")

    def run(self, command):
        assert command in HatAction.HAT_COMMANDS
        rospy.loginfo("Action {}: Command to the hat - {}".format(self.name, command))

        # Create the goal, send it to the server and wait
        goal = PlaybackGoal(primitive_name=command)
        self._hat_client.send_goal(goal)
        self.notify_action_send_goal(HatAction.PLAYBACK_ACTION_SERVER, goal)
        while self._hat_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Check the status and return appropriately
        status = self._hat_client.get_state()
        self._hat_client.wait_for_result()
        result = self._hat_client.get_result()
        self.notify_action_recv_result(HatAction.PLAYBACK_ACTION_SERVER, status, result)

        # Yield based on how we exited
        if status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted(
                action=self.name,
                status=status,
                goal=goal,
                result=result
            )
        else:
            yield self.set_aborted(
                action=self.name,
                status=status,
                goal=goal,
                result=result
            )

    def stop(self):
        self._hat_client.cancel_goal()
        self.notify_action_cancel(HatAction.PLAYBACK_ACTION_SERVER)
