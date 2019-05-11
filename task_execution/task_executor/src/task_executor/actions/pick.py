#!/usr/bin/env python
# The pick action in a task plan

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from actionlib_msgs.msg import GoalStatus
from fetch_grasp_suggestion.msg import ExecuteGraspAction, ExecuteGraspGoal, \
                                       ExecuteGraspResult
from manipulation_actions.msg import ChallengeObject


class PickAction(AbstractStep):
    """
    Assuming that grasps on a desired object have been calculated, then use the
    grasp executor at :const:`PICK_ACTION_SERVER` to execute the grasps in order
    until one succeeds
    """

    PICK_ACTION_SERVER = "/grasp_executor/execute_grasp"

    def init(self, name):
        self.name = name
        self._grasp_client = actionlib.SimpleActionClient(
            PickAction.PICK_ACTION_SERVER,
            ExecuteGraspAction
        )

        rospy.loginfo("Connecting to grasp_executor...")
        self._grasp_client.wait_for_server()
        rospy.loginfo("...grasp_executor connected")

    def run(self, object_idx, grasps, object_key, max_velocity_scaling):
        """
        The run function for this step

        Args:
            object_idx (int) : the index of the desired object in the output of \
                ``rail_segmentation``
            grasps (list of geometry_msgs/Pose) : a list of candidate grasps \
                that can be obtained from :mod:`task_executor.actions.find_grasps`
            object_key (str, int) : an identifier of the object to store in the
                kit. If a `str`, then we lookup the corresponding `int`
                identifier for the object from \
                ``manipulation_actions/ChallengeObject``
            max_velocity_scaling (double) : how fast the arm should execute

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo("Action {}: Picking up object {} at index {}".format(self.name, object_key, object_idx))

        # Resolve the argument
        if isinstance(object_key, str):
            object_key = getattr(ChallengeObject, object_key.upper())
        else:
            assert isinstance(object_key, (int, long,)), "Unknown format for object {}".format(object_key)

        # Create the template goal
        goal = ExecuteGraspGoal()
        goal.index = object_idx
        goal.grasp_pose.header.frame_id = grasps.header.frame_id
        goal.target.object = object_key

        # Iterate through all the poses, and report an error if all of them
        # failed
        status = GoalStatus.LOST
        for grasp_num, grasp_pose in enumerate(grasps.poses):
            rospy.loginfo("Action {}: Attempting grasp {}/{}"
                          .format(self.name, grasp_num + 1, len(grasps.poses)))

            goal.grasp_pose.pose = grasp_pose
            goal.grasp_pose.header.stamp = rospy.Time.now()
            goal.max_velocity_scaling_factor = max_velocity_scaling
            self._grasp_client.send_goal(goal)
            self.notify_action_send_goal(PickAction.PICK_ACTION_SERVER, goal)

            # Yield running while the client is executing
            while self._grasp_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
                yield self.set_running()

            # Check the status. Exit if we've succeeded
            status = self._grasp_client.get_state()
            self._grasp_client.wait_for_result()
            result = self._grasp_client.get_result()
            self.notify_action_recv_result(PickAction.PICK_ACTION_SERVER, status, result)

            if status == GoalStatus.SUCCEEDED or status == GoalStatus.PREEMPTED:
                break

        # Yield based on how we exited
        if status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted(
                action=self.name,
                status=status,
                goal=object_idx,
                num_grasps=len(grasps.poses),
                grasp_num=grasp_num,
                grasps=grasps,
                result=result
            )
        else:
            yield self.set_aborted(
                action=self.name,
                status=status,
                goal=object_idx,
                num_grasps=len(grasps.poses),
                grasp_num=grasp_num,
                grasps=grasps,
                result=result
            )

    def stop(self):
        self._grasp_client.cancel_goal()
        self.notify_action_cancel(PickAction.PICK_ACTION_SERVER)
