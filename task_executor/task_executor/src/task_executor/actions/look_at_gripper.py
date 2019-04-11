#!/usr/bin/env python
# The look-at-gripper action in a task plan

from __future__ import print_function, division

import numpy as np

from threading import Thread

import rospy
import tf

from task_executor.abstract_step import AbstractStep

from .look import LookAction


class LookAtGripperAction(AbstractStep):

    GRIPPER_FRAME = "gripper_link"
    HEAD_FRAME = "head_pan_link"
    BASE_FRAME = "base_link"
    HEAD_ACTION_DURATION = 0.1

    TILT_LIMITS = [-np.pi/2, np.pi/4]  # [Down, Up]
    PAN_LIMITS = [-np.pi/2, np.pi/2]  # [Left, Right]

    def init(self, name):
        self.name = name

        # Is this background behaviour enabled or is it disabled
        self.enabled = False

        # TF listener
        self._listener = tf.TransformListener()

        # The background thread to do the looking. Don't run it unless we need
        # to. At that point spawn and start the thread
        self._look_thread = None

        # The look action
        self._look_action = LookAction()

        # Initialize the sub action
        self._look_action.init('look_look_at_gripper')
        self._look_action._duration = LookAtGripperAction.HEAD_ACTION_DURATION

    def run(self, enable):
        rospy.loginfo("Action {}: {}".format(
            self.name,
            "Enable" if enable else "Disable"
        ))

        # Reset the thread
        if self._look_thread is not None:
            self.enabled = False
            self._look_thread.join()
            self._look_thread = None

        # Enable the look action and start the thread
        self.enabled = enable
        if self.enabled and self._look_thread is None:
            self._look_thread = Thread(target=self._lookat_gripper)
            self._look_thread.start()

        # There's no failure here
        yield self.set_succeeded()

    def stop(self):
        self.enabled = False
        self._look_action.stop()

    def _lookat_gripper(self):
        while not rospy.is_shutdown() and self.enabled:
            # Try to get a transform from the base to the gripper
            try:
                (trans_gripper, _) = self._listener.lookupTransform(
                    LookAtGripperAction.BASE_FRAME,
                    LookAtGripperAction.GRIPPER_FRAME,
                    rospy.Time(0)
                )
                (trans_head, _) = self._listener.lookupTransform(
                    LookAtGripperAction.BASE_FRAME,
                    LookAtGripperAction.HEAD_FRAME,
                    rospy.Time(0)
                )
            except (tf.LookupException, tf.ExtrapolationException):
                rospy.sleep(LookAtGripperAction.HEAD_ACTION_DURATION)
                continue

            # Check that we're within the cone of view
            pose = {
                'x': trans_gripper[0],
                'y': trans_gripper[1],
                'z': trans_gripper[2],
                'frame': LookAtGripperAction.BASE_FRAME
            }

            # First pan
            if not (LookAtGripperAction.PAN_LIMITS[0] <= np.arctan2(pose['y'], pose['x']) <= LookAtGripperAction.PAN_LIMITS[1]):
                rospy.sleep(LookAtGripperAction.HEAD_ACTION_DURATION)
                continue

            # Then tilt
            if not (LookAtGripperAction.TILT_LIMITS[0] <= np.arctan2((pose['z'] - trans_head[2]), pose['x']) <= LookAtGripperAction.TILT_LIMITS[1]):
                rospy.sleep(LookAtGripperAction.HEAD_ACTION_DURATION)
                continue

            # If we're OK, then send the pose to the head
            self._look_action(pose=pose)
