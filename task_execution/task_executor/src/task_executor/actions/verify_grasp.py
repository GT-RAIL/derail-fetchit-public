#!/usr/bin/env python
# Verify the gripper is holding something => position is not 0 and effort > 0

from __future__ import print_function, division

import rospy

from task_executor.abstract_step import AbstractStep

from fetch_driver_msgs.msg import GripperState
from task_execution_msgs.msg import BeliefKeys


class VerifyGraspAction(AbstractStep):
    """
    Return true if there is something in the robot's grasp, else return
    false or error out
    """

    GRIPPER_STATE_TOPIC = "/gripper_state"
    GRIPPER_CLOSED_VALUE = 0.007  # Remember to change this if the gripper changes
    SIMULATION_PARAMETER = "/use_sim_time"  # There is no effort update in simulation

    def init(self, name):
        self.name = name

        # Set a stop flag
        self._stopped = False
        self._in_simulation = rospy.get_param(VerifyGraspAction.SIMULATION_PARAMETER, False)

    def run(self, abort_on_false=False):
        rospy.loginfo("Action {}: Verifying grasp".format(self.name))

        # Set a stopped flag and then wait for a message
        self._stopped = False
        gripper_state = None
        while gripper_state is None:
            try:
                gripper_state = rospy.wait_for_message(VerifyGraspAction.GRIPPER_STATE_TOPIC, GripperState, 0.1)
                self.notify_topic_message(VerifyGraspAction.GRIPPER_STATE_TOPIC, gripper_state)
            except rospy.ROSException as e:
                pass

            if self._stopped:
                yield self.set_preempted(action=self.name)
                raise StopIteration()

            yield self.set_running()

        # Figure out whether we have grabbed something based on values and all
        # the flags
        grasped = (
            gripper_state.joints[0].position > VerifyGraspAction.GRIPPER_CLOSED_VALUE  # gripper not fully closed
            and (self._in_simulation or gripper_state.joints[0].effort > 0)  # effort > 0 unless it's simulation
        )

        # Update the beliefs
        # self.update_beliefs({ BeliefKeys.GRIPPER_HAS_OBJECT: grasped })

        # Return according to the specification
        if not grasped and abort_on_false:
            yield self.set_aborted(action=self.name, grasped=grasped)
        else:  # grasped or not abort_on_false
            yield self.set_succeeded(grasped=grasped)

    def stop(self):
        self._stopped = True
