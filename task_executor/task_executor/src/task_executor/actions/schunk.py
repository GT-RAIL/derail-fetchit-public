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
    Closes the SCHUNK machine and then once the two minutes are over, opens it
    back up
    """

    SCHUNK_ACTION_SERVER = "schunk_machine"
    SCHUNK_WAIT_DURATION = rospy.Duration(122.0)  # Give the thread two seconds more
    SCHUNK_RETRY_DURATION = rospy.Duration(0.5)   # If we fail to open, then retry every X sec

    def init(self, name):
        self.name = name
        self._schunk_client = actionlib.SimpleActionClient(
            SchunkAction.SCHUNK_ACTION_SERVER,
            SchunkMachineAction
        )

        # The background thread to open the schunk after the specified wait. If
        # such a thread already exists, then it means the SCHUNK machine is
        # closed
        self._open_thread = None

        # Initialize the action server and the beliefs action
        rospy.loginfo("Connecting to the schunk machine...")
        self._schunk_client.wait_for_server()
        rospy.loginfo("...schunk machine connected")

    def run(self):
        rospy.loginfo("Action {}: Closing the SCHUNK".format(self.name))
        self._stopped = False

        # If the schunk is already closed, then this is a noop
        if self._open_thread is not None:
            yield self.set_succeeded()
            raise StopIteration()

        # Send a close goal to the machine and if failed, return
        goal = SchunkMachineGoal(state=SchunkMachineGoal.CLOSE)
        self._schunk_client.send_goal(goal)
        self.notify_action_send_goal(SchunkAction.SCHUNK_ACTION_SERVER, goal)

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
                goal=goal,
                result=result
            )
            raise StopIteration()
        elif status == GoalStatus.ABORTED:
            yield self.set_aborted(
                action=self.name,
                status=status,
                goal=goal,
                result=result
            )
            raise StopIteration()

        # Set the schunk to a "machining" state
        self.update_beliefs({ BeliefKeys.SCHUNK_IS_MACHINING: True })

        # Setup the thread to open (and continue to reopen the schunk)
        self._open_thread = Thread(target=self._open_schunk)
        self._open_thread.start()

        # Yield a success
        yield self.set_succeeded()

    def stop(self):
        self._schunk_client.cancel_goal()
        self.notify_action_cancel(SchunkAction.SCHUNK_ACTION_SERVER)

    def _open_schunk(self):
        # Sleep until the schunk is ready to open
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and rospy.Time.now() <= start_time + SchunkAction.SCHUNK_WAIT_DURATION:
            rospy.sleep(0.5)

        # Keep trying until the schunk opens
        open_status = False
        while not rospy.is_shutdown() and not open_status:
            # Send the open goal
            goal = SchunkMachineGoal(state=SchunkMachineGoal.OPEN)
            self._schunk_client.send_goal(goal)
            self.notify_action_send_goal(SchunkAction.SCHUNK_ACTION_SERVER, goal)

            # Check the result
            self._schunk_client.wait_for_result()
            status = self._schunk_client.get_state()
            result = self._schunk_client.get_result()
            self.notify_action_recv_result(SchunkAction.SCHUNK_ACTION_SERVER, status, result)

            # If the result is a fail, sleep a bit and try again
            open_status = result.success
            if not open_status:
                rospy.sleep(SchunkAction.SCHUNK_RETRY_DURATION)
            else:
                # The schunk is done machining
                self.update_beliefs({ BeliefKeys.SCHUNK_IS_MACHINING: False })
                rospy.loginfo("Action {}: SCHUNK is now Open".format(self.name))

        # Indicate that the schunk is now open
        self._open_thread = None
