#! /usr/bin/env python

from task_executor.abstract_step import AbstractStep

# Python
from __future__ import print_function

# ROS
import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from fetchit_challenge.msg import SickCameraAction as SickCameraMsg, SickCameraResult, SickCameraGoal

from task_executor.abstract_step import AbstractStep


class TriggerSickAction(AbstractStep):
    """
    This actions  triggers the sick camera. The camera can only be successfully trigered every 5-seconds
    :param AbstractStep:
    :return:
    """

    SICK_ACTION_SERVER = "sick_camera"

    def init(self, name):
        self.name = name
        self._sick_client = actionlib.SimpleActionClient(
            TriggerSickAction.SICK_ACTION_SERVER,
            SickCameraMsg
        )

        # Initialize the action server and the beliefs action
        rospy.loginfo("Connecting to the sick camera...")
        self._sick_client.wait_for_server()
        rospy.loginfo("...sick camera connected")


    def run(self):
        """
        The run function for this step

        Args:

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo("Action {}: SICK camera triggered!".format(self.name))

        # Creates trigger goal to send to SICK action server
        goal = SickCameraGoal(trigger=SickCameraGoal.TRIG)
        self._sick_client.send_goal(goal)
        self.notify_action_send_goal(TriggerSickAction.SICK_ACTION_SERVER, goal)

        # notify the goal
        while self._sick_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        status = self._sick_client.get_state()
        self._sick_client.wait_for_result()
        result = self._sick_client.get_result()
        self.notify_action_recv_result(TriggerSickAction.SICK_ACTION_SERVER, status, result)

        if status == GoalStatus.PREEMPTED:
            yield self.set_preempted(
                action=self.name,
                status=status,
                goal=SickCameraGoal.TRIG,
                result=result
            )
            raise StopIteration()
        elif status == GoalStatus.ABORTED:
            yield self.set_aborted(
                action=self.name,
                status=status,
                goal=SickCameraGoal.TRIG,
                result=result
            )
            raise StopIteration()
        # If the result is a fail, sleep a bit and try again
        trigger_status = result.success
        if not trigger_status:
            yield self.set_aborted(
                action=self.name,
                status=status,
                goal=SickCameraGoal.TRIG,
                result=result
            )
            raise StopIteration()

        # Yield a success
        yield self.set_succeeded()

    def stop(self):
        self._sick_client.cancel_goal()
        self.notify_action_cancel(TriggerSickAction.SICK_ACTION_SERVER)