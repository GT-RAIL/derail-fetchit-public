import rospy
import rospkg

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose


class StateRecorder(object):
    r = rospkg.RosPack()
    PACKAGE_PATH = r.get_path('data_recorder')

    def __init__(self, filename, rate, joint_state_topic = "/joint_states", eef_state_topic="/eef_pose"):
        """
        Records joint data to a file at a specified rate.
        """
        self._filename = self.PACKAGE_PATH + '/data/' + filename
        self._raw_rate = rate
        self._rate = rospy.Rate(rate)
        self._start_time = rospy.get_time()
        self._done = False

        self._eef_state = []
        self._joint_state = []

        self._joint_state_subscriber = rospy.Subscriber(joint_state_topic, JointState, self._cb_joint_state, queue_size=1)
        # self._eef_state_subscriber = rospy.Subscriber(eef_state_topic, Pose, self._cb_eef_state, queue_size=1)

    def _cb_joint_state(self, msg):
        if len(msg.position) > 3:
            self._joint_state = msg.position[6:]

    def _cb_eef_state(self, msg):
        self._eef_state = [msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

    def _time_stamp(self):
        return rospy.get_time() - self._start_time

    def stop(self):
        """
        Stop recording.
        """
        self._done = True

    def done(self):
        """
        Return whether or not recording is done.
        """
        if rospy.is_shutdown():
            self.stop()
        return self._done

    def record(self):
        """
        Records the current joint positions to a csv file if outputFilename was
        provided at construction this function will record the latest set of
        joint angles in a csv format.

        If a file exists, the function will overwrite existing file.
        """
        if self._filename:
            with open(self._filename, 'w') as f:
                temp_str = '\n'
                while not self.done():
                    if self._joint_state:
                        f.write("%f " % (self._time_stamp(),))
                        f.write(' '.join([str(x) for x in self._joint_state]) + temp_str)
                        # f.write(' '.join([str(x) for x in self._eef_state]) + temp_str)
			#f.write(' '.join([str(x) for x in self._joint_state]) + temp_str)
                        self._rate.sleep()
