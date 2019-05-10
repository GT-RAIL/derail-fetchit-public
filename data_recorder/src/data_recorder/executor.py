import rospy
import rospkg
import numpy
import operator

from numpy.linalg import norm

import geometry_msgs.msg
import sensor_msgs.msg

import control_msgs.msg
import actionlib
import math
import trajectory_msgs.msg

from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface


class TrajectoryExecutor:
    JOINT_STATE_TOPIC = '/joint_states'
    JOINT_VEL_ACTION = '/arm_controller/follow_joint_trajectory'           # This Action Call smooths the trajectory
    # Minimum threshold for a point to be included in trajectory. Identical adjacent points cause velocity control failure
    MIN_JOINT_DIFF = 0.001

    r = rospkg.RosPack()
    PACKAGE_PATH = r.get_path('data_recorder')

    def __init__(self, filename='test3.txt', debug=False):
        self.debug = debug
        self.filename = filename
        self.jointDataPath = self.PACKAGE_PATH + '/data/' + self.filename

        self.jointStateArm = None

        self.trajActionGoal = control_msgs.msg.FollowJointTrajectoryGoal()
        self.trajActionGoal.trajectory.joint_names.append('shoulder_pan_joint')
        self.trajActionGoal.trajectory.joint_names.append('shoulder_lift_joint')
        self.trajActionGoal.trajectory.joint_names.append('upperarm_roll_joint')
        self.trajActionGoal.trajectory.joint_names.append('elbow_flex_joint')
        self.trajActionGoal.trajectory.joint_names.append('forearm_roll_joint')
        self.trajActionGoal.trajectory.joint_names.append('wrist_flex_joint')
        self.trajActionGoal.trajectory.joint_names.append('wrist_roll_joint')

        self.trajActionClient = actionlib.SimpleActionClient(self.JOINT_VEL_ACTION,
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

    def loadJointTrajectory(self):
        """
        Loads a pre-computed joint trajectory if required
        """
        time = numpy.loadtxt(self.jointDataPath, delimiter=' ', usecols=(0,))
        data = numpy.loadtxt(self.jointDataPath, delimiter=' ', usecols=(1,2,3,4,5,6,7))

        time = (time - time[0])

        currJoint = [100000]*7
        currTime = 0.0
        dt = time[2] - time[1]

        for i in range(len(data)):
            if norm(data[i]-currJoint) >= self.MIN_JOINT_DIFF:
                tempTrajPoint = trajectory_msgs.msg.JointTrajectoryPoint()
                tempTrajPoint.positions = data[i]
                # tempTrajPoint.time_from_start = rospy.Duration(time[i])
                tempTrajPoint.time_from_start = rospy.Duration(currTime)
                self.trajActionGoal.trajectory.points.append(tempTrajPoint)
                currJoint = data[i]
                currTime = currTime + dt

    def executeTrajectory(self):
        """
        Passes the trajectory to controller
        """
        # self.gotoInit()
        self.trajActionGoal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)

        rospy.on_shutdown(self.preemptExecution)

        self.trajActionClient.send_goal(self.trajActionGoal)
        self.trajActionClient.wait_for_result()

        if self.debug:
            jointStateSub = rospy.Subscriber(self.JOINT_STATE_TOPIC,
                                             sensor_msgs.msg.JointState,
                                             self.jointStateCallback)

            try:
                rospy.wait_for_message(self.JOINT_STATE_TOPIC,
                                       sensor_msgs.msg.JointState, timeout=2)
            except rospy.ROSException, e:
                rospy.logerr("Joint State Callback Time Out: %s", str(e))


            finalError = map(operator.sub, self.jointStateArm, list(self.jointTrajectory[-1].positions))

            rospy.loginfo('Goal Point Error: ' + ','.join([str(x) for x in finalError]))

        trajActionResult = (self.trajActionClient.get_result().error_code == 0)

        if trajActionResult:
            return True
        else:
            return False


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
        init = self.trajActionGoal.trajectory.points[0].positions
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
