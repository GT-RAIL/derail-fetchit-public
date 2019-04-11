#!/usr/bin/env python
# Send a belief update when the torso is raised

from __future__ import print_function, division

import rospy

from sensor_msgs.msg import JointState
from task_execution_msgs.msg import BeliefKeys

from task_monitor.monitoring import AbstractBeliefMonitor


# The class definition

class TorsoRaisedMonitor(AbstractBeliefMonitor):
    """
    Monitor the torso state and fire a belief update when it is raised vs. when
    it is not.
    """

    JOINT_STATES_TOPIC = "/joint_states"
    TORSO_JOINT_NAME = "torso_lift_joint"
    TORSO_JOINT_RAISED_THRESHOLD = 0.04  # 4 cm or more means the torso is raised

    def __init__(self):
        super(TorsoRaisedMonitor, self).__init__()

        # Setup the monitor
        self._joint_states_sub = rospy.Subscriber(
            TorsoRaisedMonitor.JOINT_STATES_TOPIC,
            JointState,
            self._on_joint_states
        )

    def _on_joint_states(self, msg):
        try:
            idx = msg.name.index(TorsoRaisedMonitor.TORSO_JOINT_NAME)
            return self.update_beliefs({
                BeliefKeys.TORSO_RAISED: (msg.position[idx] >= TorsoRaisedMonitor.TORSO_JOINT_RAISED_THRESHOLD)
            })
        except ValueError as e:
            pass


# When running the monitor in standalone mode
if __name__ == '__main__':
    rospy.init_node('torso_raised_monitor')
    monitor = TorsoRaisedMonitor()
    rospy.spin()
