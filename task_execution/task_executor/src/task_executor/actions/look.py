#!/usr/bin/env python
# The look action in a task plan

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from control_msgs.msg import PointHeadAction, PointHeadGoal
from actionlib_msgs.msg import GoalStatus
from task_execution_msgs.srv import GetArmGripperPose


class LookAction(AbstractStep):
    """
    Point the head at a pose in the world. Uses the :const:`HEAD_ACTION_SERVER`
    """

    HEAD_ACTION_SERVER = "/head_controller/point_head"
    HEAD_ACTION_DURATION = 0.5

    ARM_GRIPPER_POSES_SERVICE_NAME = "/database/arm_gripper_pose"

    def init(self, name):
        self.name = name
        self._look_client = actionlib.SimpleActionClient(
            LookAction.HEAD_ACTION_SERVER,
            PointHeadAction
        )
        self._get_arm_gripper_pose_srv = rospy.ServiceProxy(LookAction.ARM_GRIPPER_POSES_SERVICE_NAME, GetArmGripperPose)

        self._duration = LookAction.HEAD_ACTION_DURATION

        rospy.loginfo("Connecting to head_controller...")
        self._look_client.wait_for_server()
        rospy.loginfo("...head_controller connected")

        rospy.loginfo("Connecting to database services...")
        self._get_arm_gripper_pose_srv.wait_for_service()
        rospy.loginfo("...database services connected")

    def run(self, pose):
        """
        The run function for this step

        Args:
            pose (str, dict) :
                The poses to look at. If the type is:

                * str. Then if the string starts with
                    * `gripper_poses`, get a ``geometry_msgs/PoseStamped`` \
                        from :const:`ARM_GRIPPER_POSES_SERVICE_NAME` and point \
                        the head at that 3D position in that pose
                * dict. Then if the keys of the dict are
                    * `position, orientation, frame`, extract the 3D position \
                        from the pose and look there
                    * `x, y, z, frame`, use the defined 3D position as is

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.logdebug("Action {}: Looking at point: {}".format(self.name, pose))

        # Parse out the pose
        parsed_pose = self._parse_pose(pose)
        if parsed_pose is None:
            rospy.logerr("Action {}: FAIL. Unknown Format: {}".format(self.name, pose))
            raise KeyError(self.name, "Unknown Format", pose)

        # Create and send the goal pose
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = parsed_pose['frame']
        goal.target.point.x = parsed_pose['x']
        goal.target.point.y = parsed_pose['y']
        goal.target.point.z = parsed_pose['z']
        goal.min_duration = rospy.Duration(self._duration)
        self._look_client.send_goal(goal)
        self.notify_action_send_goal(LookAction.HEAD_ACTION_SERVER, goal)

        # Yield an empty dict while we're executing
        while self._look_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Wait for a result and yield based on how we exited
        status = self._look_client.get_state()
        self._look_client.wait_for_result()
        result = self._look_client.get_result()
        self.notify_action_recv_result(LookAction.HEAD_ACTION_SERVER, status, result)

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
        self._look_client.cancel_goal()
        self.notify_action_cancel(LookAction.HEAD_ACTION_SERVER)

    def _parse_pose(self, pose):
        parsed_pose = None

        if isinstance(pose, str):
            # This is a reference to poses stored in the DB
            db_name, pose = pose.split('.', 1)
            if db_name == 'gripper_poses':
                pose_stamped = self._get_arm_gripper_pose_srv(pose).pose
                self.notify_service_called(LookAction.ARM_GRIPPER_POSES_SERVICE_NAME)
                parsed_pose = {
                    'x': pose_stamped.pose.position.x,
                    'y': pose_stamped.pose.position.y,
                    'z': pose_stamped.pose.position.z,
                    'frame': pose_stamped.header.frame_id,
                }
        elif isinstance(pose, dict) and pose.has_key('x'):
            parsed_pose = pose
        elif isinstance(pose, dict) and pose.has_key('position'):
            parsed_pose = {
                'frame': pose['frame'],
                'x': pose['position']['x'],
                'y': pose['position']['y'],
                'z': pose['position']['z'],
            }

        return parsed_pose
