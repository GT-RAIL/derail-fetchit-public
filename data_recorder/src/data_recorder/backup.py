#!/usr/bin/env python

import rospy
import rospkg
import numpy
import operator

from numpy.linalg.linalg import dot
from numpy.linalg import norm

import geometry_msgs.msg
import sensor_msgs.msg

import control_msgs.msg
import actionlib
import math
import trajectory_msgs.msg


class TrajectoryExecutor:
    JOINT_STATE_TOPIC = '/joint_states'
    JOINT_VEL_ACTION = '/arm_controller/follow_joint_trajectory'           # This Action Call smooths the trajectory
    # Minimum threshold for a point to be included in trajectory. Identical adjacent points cause velocity control failure
    MIN_JOINT_DIFF = 0.0

    r = rospkg.RosPack()
    PACKAGE_PATH = r.get_path('data_recorder')

    def __init__(self, filename='test3.txt', debug=False):

        self.filename = filename
        self.jointDataPath = self.PACKAGE_PATH + '/data/' + self.filename

        self.jointStateArm = None

        self.jointTrajectory = []
        self.timeList = []


        self.jointTrajAction = self.JOINT_VEL_ACTION
        self.trajActionClient = actionlib.SimpleActionClient(self.jointTrajAction,
                                                             control_msgs.msg.FollowJointTrajectoryAction)
        try:
            self.trajActionClient.wait_for_server()
        except rospy.ROSException, e:
            rospy.logerr("Timed Out waiting for Trajectory Follower Action Server: %s", str(e))


        jointStateSub = rospy.Subscriber(self.JOINT_STATE_TOPIC,
                                         sensor_msgs.msg.JointState,
                                         self.jointStateCallback)
        try:
            rospy.wait_for_message(self.JOINT_STATE_TOPIC,
                                   sensor_msgs.msg.JointState, timeout=2)
        except rospy.ROSException, e:
            rospy.logerr("Joint State Callback Time Out: %s", str(e))

        self.loadJointTrajectory()
        # self.gotoInit()
        # self.executeTrajectory()


    def jointStateCallback(self, jointStateMsg):
        # Populating joint names
        if len(jointStateMsg.position)>3:
            self.jointStateArm = [jointStateMsg.position[6],
                                  jointStateMsg.position[7],
                                  jointStateMsg.position[8],
                                  jointStateMsg.position[9],
                                  jointStateMsg.position[10],
                                  jointStateMsg.position[11],
                                  jointStateMsg.position[12]]


    def loadJointTrajectory(self):
        time = numpy.loadtxt(self.jointDataPath, delimiter=' ', usecols=(0,))
        data = numpy.loadtxt(self.jointDataPath, delimiter=' ', usecols=(1,2,3,4,5,6,7))

        time = (time - time[0])

        trajActionGoal = control_msgs.msg.FollowJointTrajectoryGoal()

        trajActionGoal.trajectory.joint_names.append('shoulder_pan_joint')
        trajActionGoal.trajectory.joint_names.append('shoulder_lift_joint')
        trajActionGoal.trajectory.joint_names.append('upperarm_roll_joint')
        trajActionGoal.trajectory.joint_names.append('elbow_flex_joint')
        trajActionGoal.trajectory.joint_names.append('forearm_roll_joint')
        trajActionGoal.trajectory.joint_names.append('wrist_flex_joint')
        trajActionGoal.trajectory.joint_names.append('wrist_roll_joint')

        currJoint = [None]*7

        for i in range(len(data)):
            if norm(data[i]-currJoint) >= self.MIN_JOINT_DIFF:
                tempTrajPoint = trajectory_msgs.msg.JointTrajectoryPoint()
                tempTrajPoint.positions = data[i]
                tempTrajPoint.time_from_start = rospy.Duration(time[i])
                trajActionGoal.trajectory.points.append(tempTrajPoint)
                currJoint = data[i]






        """
        Loads a pre-computed joint trajectory if required
        """
        rospy.loginfo('Loading Joint Data ...')
        time = numpy.loadtxt(self.jointDataPath, delimiter=' ', usecols=(0,))
        data = numpy.loadtxt(self.jointDataPath, delimiter=' ', usecols=(1,2,3,4,5,6,7))

        jointList = []
        jointList.append(data[0,:])

        time = (time - time[0])

        self.timeList.append(time[0])

        # Check to append joint values only when they are different
        for i in range(1, len(data)):
            currJoint = jointList[-1]
            jointDiff = norm(data[i]-currJoint)

            if jointDiff >= self.MIN_JOINT_DIFF:
                jointList.append(data[i])
                self.timeList.append(time[i])

        # Populating joint trajectory
        for i in range(len(jointList)):
            trajPoint = trajectory_msgs.msg.JointTrajectoryPoint(positions=jointList[i])
            self.jointTrajectory.append(trajPoint)


    def preemptExecution(self):
        if (self.trajActionClient.gh is not None and
                self.trajActionClient.get_state() == actionlib.GoalStatus.ACTIVE):
            self.trajActionClient.cancel_goal()

        #delay to allow for terminating handshake
        rospy.sleep(0.1)

    def gotoInit(self):
        move_group = MoveGroupInterface("arm", "base_link")
        planning_scene = PlanningSceneInterface("base_link")
        joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                       "elbow_flex_joint", "forearm_roll_joint",
                       "wrist_flex_joint", "wrist_roll_joint"]

        # Plans the joints in joint_names to angles in pose
        init = self.jointTrajectory[0].positions
        move_group.moveToJointPosition(joint_names, init, wait=False)
        move_group.get_move_action().wait_for_result()
        result = move_group.get_move_action().get_result()

        if result:
            # Checking the MoveItErrorCode
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Moved to Init")
            else:
                # If you get to this point please search for:
                # moveit_msgs/MoveItErrorCodes.msg
                rospy.logerr("Arm goal in state: %s",
                             move_group.get_move_action().get_state())
        else:
            rospy.logerr("MoveIt! failure no result returned.")

    def executeTrajectory(self):
        """
        Passes the trajectory to controller
        """
        trajActionGoal = control_msgs.msg.FollowJointTrajectoryGoal()

        trajActionGoal.trajectory.joint_names.append('shoulder_pan_joint')
        trajActionGoal.trajectory.joint_names.append('shoulder_lift_joint')
        trajActionGoal.trajectory.joint_names.append('upperarm_roll_joint')
        trajActionGoal.trajectory.joint_names.append('elbow_flex_joint')
        trajActionGoal.trajectory.joint_names.append('forearm_roll_joint')
        trajActionGoal.trajectory.joint_names.append('wrist_flex_joint')
        trajActionGoal.trajectory.joint_names.append('wrist_roll_joint')

        trajActionGoal.trajectory.points = []
        for j in range(len(self.jointTrajectory)):
            tempTrajPoint = trajectory_msgs.msg.JointTrajectoryPoint()
            tempTrajPoint.positions = self.jointTrajectory[j].positions
            tempTrajPoint.time_from_start = rospy.Duration(self.timeList[j])
            trajActionGoal.trajectory.points.append(tempTrajPoint)

        trajActionGoal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)

        rospy.on_shutdown(self.preemptExecution)

        self.trajActionClient.send_goal(trajActionGoal)
        self.trajActionClient.wait_for_result()

        if debug:
            jointStateSub = rospy.Subscriber(self.JOINT_STATE_TOPIC,
                                             sensor_msgs.msg.JointState,
                                             self.jointStateCallback)

            try:
                rospy.wait_for_message(self.JOINT_STATE_TOPIC,
                                       sensor_msgs.msg.JointState, timeout=2)
            except rospy.ROSException, e:
                rospy.logerr("Joint State Callback Time Out: %s", str(e))


            finalError = map(operator.sub, self.jointStateArm, list(self.jointTrajectory[-1].positions))

            rospy.loginfo('Final Point Error: ' + ','.join([str(x) for x in finalError]))



        trajActionResult = (self.trajActionClient.get_result().error_code == 0)

        if trajActionResult:
            return True
        else:
            return False
