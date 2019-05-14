#!/usr/bin/env python
# Close and open the SCHUNK machine a couple of minutes later

from __future__ import print_function, division

from threading import Thread

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from fetchit_challenge.msg import SchunkMachineAction, SchunkMachineGoal
from task_execution_msgs.msg import BeliefKeys

from task_executor.abstract_step import AbstractStep


class SchunkAction(AbstractStep):
    """
    Depending on the command sent to the SCHUNK machine, this action opens or
    closes it. If the command fails, the action indicates a failure too
    """

    SCHUNK_ACTION_SERVER = "schunk_machine"

    def init(self, name):
        self.name = name
        self._schunk_client = actionlib.SimpleActionClient(
            SchunkAction.SCHUNK_ACTION_SERVER,
            SchunkMachineAction
        )

        # Initialize the action server and the beliefs action
        rospy.loginfo("Connecting to the schunk machine...")
        self._schunk_client.wait_for_server()
        rospy.loginfo("...schunk machine connected")

    def run(self, command):
        """
        The run function for this step

        Args:
            command (str) : Must be either `open` or `close`

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        if not isinstance(command, str) or command.lower() not in ['close', 'open']:
            rospy.logerr("Action: {}. FAIL. Unrecognized: {}".format(self.name, command))
            raise KeyError(self.name, "Unrecognized", command)

        rospy.loginfo("Action {}: SCHUNK {}".format(self.name, command))

        # Create and send the goal pose
        if command.lower() == 'close':
            goal = SchunkMachineGoal(state=SchunkMachineGoal.CLOSE)
        elif command.lower() == 'open':
            goal = SchunkMachineGoal(state=SchunkMachineGoal.OPEN)

        # Send a goal
        self._schunk_client.send_goal(goal)
        self.notify_action_send_goal(SchunkAction.SCHUNK_ACTION_SERVER, goal)

        # notify the goal
        while self._schunk_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Wait for a status. If it was a success, then setup the open thread
        status = self._schunk_client.get_state()
        self._schunk_client.wait_for_result()
        result = self._schunk_client.get_result()
        self.notify_action_recv_result(SchunkAction.SCHUNK_ACTION_SERVER, status, result)

        if status == GoalStatus.PREEMPTED:
            yield self.set_preempted(
                action=self.name,
                status=status,
                goal=command,
                result=result
            )
            raise StopIteration()
        elif status == GoalStatus.ABORTED:
            yield self.set_aborted(
                action=self.name,
                status=status,
                goal=command,
                result=result
            )
            raise StopIteration()

        # If the result is a fail, sleep a bit and try again
        open_status = result.success
        if not open_status:
            yield self.set_aborted(
                action=self.name,
                status=status,
                goal=command,
                result=result
            )
            raise StopIteration()

        # Set the schunk to a "machining" state
        if command.lower() == 'close':
            self.update_beliefs({
                BeliefKeys.SCHUNK_IS_MACHINING: True,
                BeliefKeys.SCHUNK_IS_READY: False,
            })
        else:
            self.update_beliefs({ BeliefKeys.SCHUNK_IS_MACHINING: False })

        # Yield a success
        yield self.set_succeeded()

    def stop(self):
        self._schunk_client.cancel_goal()
        self.notify_action_cancel(SchunkAction.SCHUNK_ACTION_SERVER)
