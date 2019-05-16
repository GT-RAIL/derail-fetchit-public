#!/usr/bin/env python
# Pick up a kit from the table

from __future__ import print_function

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from geometry_msgs.msg import PoseStamped
from manipulation_actions.msg import SchunkGraspAction, SchunkGraspGoal, SchunkGraspResult
from actionlib_msgs.msg import GoalStatus
from task_execution_msgs.srv import GetArmGripperPose


class GraspSchunkGearAction(AbstractStep):
    """
    Grasp the gear inserted in the schunk. This action requires that the pose of the schunk is known,
    preferably through :mod:`task_executor.actions.detect_schunk`
    """

    GRASP_SCHUNK_GEAR_ACTION_SERVER = '/schunk_gear_grasper/grasp_schunk_gear'
    ARM_GRIPPER_POSES_SERVICE_NAME = "/database/arm_gripper_pose"

    def init(self, name):
        self.name = name

        self._grasp_client = actionlib.SimpleActionClient(
            GraspSchunkGearAction.GRASP_SCHUNK_GEAR_ACTION_SERVER,
            SchunkGraspAction
        )

        self._get_arm_gripper_poses_srv = rospy.ServiceProxy(
            GraspSchunkGearAction.ARM_GRIPPER_POSES_SERVICE_NAME,
            GetArmGripperPose
        )

        rospy.loginfo("Connecting to grasp_schunk_gear...")
        self._grasp_client.wait_for_server()
        rospy.loginfo("...grasp_schunk_gear connected")

        rospy.loginfo("Connecting to database...")
        self._get_arm_gripper_poses_srv.wait_for_service()
        rospy.loginfo("...database connected")

    def run(self, pose):
        """
        The run function for this step

        Args:
            pose (str, list, tuple, dict) :
                The joint pose for grasping the gear in the schunk. If the type is:

                * str. Then if the string starts with
                    * `gripper_poses`, get a ``geometry_msgs/PoseStamped`` \
                        from :const:`ARM_GRIPPER_POSES_SERVICE_NAME` and move \
                        the end effector to that pose
                * dict. Then if the keys of the dict are
                    * `position, orientation, frame`, create a \
                        ``geometry_msgs/PoseStamped`` from the dictionary and \
                        move the gripper to the desired pose

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo("Action {}: Grasping the gear in the schunk".format(self.name))

        parsed_pose = self._parse_poses(pose)
        if parsed_pose is None:
            rospy.logerr("Action {}: FAIL. Unknown Format: {}".format(self.name, pose))
            raise KeyError(self.name, "Unknown Format", pose)

        rospy.logdebug("Action {}: Moving to gripper pose: {}".format(self.name, parsed_pose))

        # Create and send the goal
        goal = SchunkGraspGoal()
        goal.grasp_pose = parsed_pose
        self._grasp_client.send_goal(goal)
        self.notify_action_send_goal(
            GraspSchunkGearAction.GRASP_SCHUNK_GEAR_ACTION_SERVER, goal
        )

        # Yield while we're executing
        while self._grasp_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Wait for a result and yield based on how we exited
        status = self._grasp_client.get_state()
        self._grasp_client.wait_for_result()
        result = self._grasp_client.get_result()
        self.notify_action_recv_result(
            GraspSchunkGearAction.GRASP_SCHUNK_GEAR_ACTION_SERVER, status, result
        )

        if status == GoalStatus.SUCCEEDED and result.error_code == SchunkGraspResult.SUCCESS:
            yield self.set_succeeded()
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted(
                action=self.name,
                status=status,
                result=result,
            )
        else:
            yield self.set_aborted(
                action=self.name,
                status=status,
                result=result,
            )

    def stop(self):
        self._grasp_client.cancel_goal()
        self.notify_action_cancel(GraspSchunkGearAction.GRASP_SCHUNK_GEAR_ACTION_SERVER)

    def _parse_poses(self, pose):
        """
        Parses out a meaningful pose from the incoming argument to the
        function.

        Params:
        - string. Startswith
            - gripper_poses: get PoseStamped from ARM_GRIPPER_POSES_SERVICE_NAME
        - dictionary with the keys position, orientation, & frame: PoseStamped

        Returns:
        - PoseStamped()
        """
        parsed_pose = None

        if isinstance(pose, str):
            # This is a reference to stored poses in the DB
            db_name, pose = pose.split('.', 1)
            if db_name == 'gripper_poses':
                parsed_pose = self._get_arm_gripper_poses_srv(pose).pose
                self.notify_service_called(GraspSchunkGearAction.ARM_GRIPPER_POSES_SERVICE_NAME)
        elif isinstance(pose, dict) \
                and pose.has_key("position") and pose.has_key("orientation") and pose.has_key("frame"):
            # YAML definition of a PoseStamped EEF pose
            parsed_pose = PoseStamped()
            parsed_pose.header.frame_id = pose['frame']
            parsed_pose.pose.position.x = pose['position']['x']
            parsed_pose.pose.position.y = pose['position']['y']
            parsed_pose.pose.position.z = pose['position']['z']
            parsed_pose.pose.orientation.x = pose['orientation']['x']
            parsed_pose.pose.orientation.y = pose['orientation']['y']
            parsed_pose.pose.orientation.z = pose['orientation']['z']
            parsed_pose.pose.orientation.w = pose['orientation']['w']

        return parsed_pose
