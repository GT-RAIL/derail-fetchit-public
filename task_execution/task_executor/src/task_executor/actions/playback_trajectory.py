#!/usr/bin/env python
# The playback trajectory action in the task plan

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from actionlib_msgs.msg import GoalStatus
from data_recorder.msg import PlaybackAction, PlaybackGoal


# The class definition

class PlaybackTrajectoryAction(AbstractStep):
    """
    Playback a recorded trajectory from filename

    To add a trajectory, place trajectory file in "{data_recorder}/data",
    add command name in DELIVERY_COMMANDS, and point to file in 
    "{data_recorder}/scripts/playback_executor.py"

    """

    PLAYBACK_ACTION_SERVER = "/playback_primitive"
    DELIVERY_COMMANDS = ["place_complete_bin"]

    def init(self, name):
        self.name = name
        self._playback_client = actionlib.SimpleActionClient(
            PlaybackTrajectoryAction.PLAYBACK_ACTION_SERVER,
            PlaybackAction
        )

        rospy.loginfo("Connecting to playback executor...")
        self._playback_client.wait_for_server()
        rospy.loginfo("...playback executor connected")

    def run(self, command):
        """
        The run function for this step

        Args:
            command (str) : Must be a valid trajectory name in DELIVERY_COMMANDS

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        assert cofrommmand in PlaybackTrajectoryAction.DELIVERY_COMMANDS
        rospy.loginfo("Action {}: Command to arm - {}".format(self.name, command))

        # Create the goal, send it to the server and wait
        goal = PlaybackGoal(primitive_name=command)
        self._playback_client.send_goal(goal)
        self.notify_action_send_goal(PlaybackTrajectoryAction.PLAYBACK_ACTION_SERVER, goal)
        while self._playback_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Check the status and return appropriately
        status = self._playback_client.get_state()
        self._playback_client.wait_for_result()
        result = self._playback_client.get_result()
        self.notify_action_recv_result(PlaybackTrajectoryAction.PLAYBACK_ACTION_SERVER, status, result)

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
        self._playback_client.cancel_goal()
        self.notify_action_cancel(PlaybackTrajectoryAction.PLAYBACK_ACTION_SERVER)
