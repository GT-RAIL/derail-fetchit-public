#!/usr/bin/env python
# The pick action in a task plan

import rospy
import actionlib
from std_srvs.srv import Empty

from task_executor.abstract_step import AbstractStep

from actionlib_msgs.msg import GoalStatus
from fetch_grasp_suggestion.msg import ExecuteGraspAction, ExecuteGraspGoal, \
                                       ExecuteGraspResult
from manipulation_actions.msg import ChallengeObject

from rail_semantic_grasping.msg import SemanticObjectList
# import rospkg

class SemPickAction(AbstractStep):
    """
    Assuming that grasps on a desired object have been calculated, then use the
    grasp executor at :const:`PICK_ACTION_SERVER` to execute the grasps in order
    until one succeeds
    """

    PICK_ACTION_SERVER = "/grasp_executor/execute_grasp"
    SEMANTIC_GRASP_SERVICE = "/semantic_grasp_suggestion/get_semantic_grasps"
    SEMANTIC_OBJECTS_TOPIC = "/semantic_grasp_suggestion/semantic_objects_with_grasps"

    def init(self, name):
        self.name = name
        self._pick_client = actionlib.SimpleActionClient(
            SemPickAction.PICK_ACTION_SERVER,
            ExecuteGraspAction
        )
        self._grasp_srv = rospy.ServiceProxy(
            SemPickAction.SEMANTIC_GRASP_SERVICE,
            Empty
        )

        rospy.loginfo("Connecting to grasp_executor...")
        self._pick_client.wait_for_server()
        rospy.loginfo("...grasp_executor connected")
        rospy.loginfo("Connecting to semantic grasp retriever...")
        rospy.wait_for_service(SemPickAction.SEMANTIC_GRASP_SERVICE)
        rospy.loginfo("...semantic grasp retriever connected")

    def run(self, object_idx, object_key, max_velocity_scaling):
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

        # get a new semantic object
        self._grasp_srv()

        # retrieves the semantic object from the topic
        sem_object_list = rospy.wait_for_message(SemPickAction.SEMANTIC_OBJECTS_TOPIC,SemanticObjectList,20.0)
        sem_object = sem_object_list.objects[0]

        # Resolve the argument
        if isinstance(object_key, str):
            object_key = getattr(ChallengeObject, object_key.upper())
        else:
            assert isinstance(object_key, (int, long,)), "Unknown format for object {}".format(object_key)

        # extract list of grasps formatted for grasp executor
        sem_grasps = []
        for grasp in sem_object.grasps:
            #if grasp.grasp_part_affordance == "handle":
            sem_grasps.append(grasp.grasp_pose)

        if len(sem_grasps) == 0:
            rospy.loginfo("No grasps matching criteria.")

        # Create the template goal
        goal = ExecuteGraspGoal()
        goal.index = object_idx
        goal.grasp_pose.header.frame_id = sem_object_list.header.frame_id
        goal.grasp_pose.header.stamp = sem_object_list.header.stamp
        goal.target.object = object_key

        # Iterate through all the poses, and report an error if all of them
        # failed
        status = GoalStatus.LOST
        # start_moveit_time = rospy.Time.now()
        # end_moveit_time = rospy.Time.now()
        # moveit_attempts = 0
        for grasp_num, grasp_pose in enumerate(sem_grasps):
            if grasp_num > 5:
                rospy.loginfo("Stopped trying at 5th grasp.")

            rospy.loginfo("Action {}: Attempting grasp {}/{}"
                          .format(self.name, grasp_num + 1, len(sem_grasps)))

            # moveit_attempts += 1

            goal.grasp_pose.pose = grasp_pose
            goal.grasp_pose.header.stamp = rospy.Time.now()
            goal.max_velocity_scaling_factor = max_velocity_scaling
            self._pick_client.send_goal(goal)
            self.notify_action_send_goal(SemPickAction.PICK_ACTION_SERVER, goal)

            # Yield running while the client is executing
            while self._pick_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
                yield self.set_running()

            # Check the status. Exit if we've succeeded
            status = self._pick_client.get_state()
            self._pick_client.wait_for_result()
            result = self._pick_client.get_result()
            self.notify_action_recv_result(SemPickAction.PICK_ACTION_SERVER, status, result)

            if status == GoalStatus.SUCCEEDED or status == GoalStatus.PREEMPTED:
                break

            # end_moveit_time = rospy.Time.now()

        # logfile = open(rospkg.RosPack().get_path('task_monitor') + '/data/moveit_times_pick.txt', 'a')
        # logfile.write(str(moveit_attempts) + ", " + str((end_moveit_time - start_moveit_time).to_sec()) + "\n")
        # logfile.close()

        # Yield based on how we exited
        if status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted(
                action=self.name,
                status=status,
                goal=object_idx,
                num_grasps=len(sem_grasps),
                grasp_num=grasp_num,
                grasps=sem_grasps,
                result=result
            )
        else:
            yield self.set_aborted(
                action=self.name,
                status=status,
                goal=object_idx,
                num_grasps=len(sem_grasps),
                grasp_num=grasp_num,
                grasps=sem_grasps,
                result=result
            )

    def stop(self):
        self._pick_client.cancel_goal()
        self.notify_action_cancel(SemPickAction.PICK_ACTION_SERVER)
