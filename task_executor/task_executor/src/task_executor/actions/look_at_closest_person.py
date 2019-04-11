#!/usr/bin/env python
# The look-at-closest-person action in a task plan

import numpy as np

from threading import Thread

import rospy

from task_executor.abstract_step import AbstractStep

from rail_people_detection_msgs.msg import Person, DetectionContext

from .look import LookAction


class LookAtClosestPersonAction(AbstractStep):

    CLOSEST_PERSON_TOPIC = "/rail_people_detector/closest_person"
    POSITION_CHANGE_HEAD_FOLLOW_THRESHOLD = 0.02
    HEAD_ACTION_DURATION = 0.1

    def init(self, name):
        self.name = name

        # Is this background behaviour enabled or is it disabled?
        self.enabled = False

        # Variable to keep track of the person that we're looking at
        self._closest_person = None
        self._desired_person_id = None
        self._last_look_pose = None  # Don't want erratic look behaviour

        self._closest_person_sub = rospy.Subscriber(
            LookAtClosestPersonAction.CLOSEST_PERSON_TOPIC,
            Person,
            self._on_closest_person
        )

        # The background thread to do the looking. Don't run it unless we need
        # to. At that point spawn and start the thread
        self._look_thread = None

        # The look action
        self._look_action = LookAction()

        # Initialize the sub action
        self._look_action.init('look_look_at_closest_person')
        self._look_action._duration = LookAtClosestPersonAction.HEAD_ACTION_DURATION

    def run(self, enable, person_id=""):
        rospy.loginfo("Action {}: {}".format(
            self.name,
            "Enable{}".format(
                ". Looking at {}".format(person_id) if person_id else ""
            ) if enable else "Disable"
        ))

        # Reset the variables if this is a new person
        if self._desired_person_id != person_id:
            if self._look_thread is not None:
                self.enabled = False
                self._look_thread.join()
                self._look_thread = None
            self._closest_person = None
            self._last_look_pose = None

        # Set the variables
        self._desired_person_id = person_id
        self.enabled = enable
        if self.enabled and self._look_thread is None:
            self._look_thread = Thread(target=self._lookat_person)
            self._look_thread.start()

        # There's no failure here
        yield self.set_succeeded()

    def stop(self):
        self.enabled = False
        self._look_action.stop()

    def _on_closest_person(self, msg):
        # If we're disabled, then there's nothing to do here
        if not self.enabled:
            return

        # If this person is the one we're meant to be tracking or if we're
        # meant to track anyone, then add this person
        if not self._desired_person_id or msg.id == self._desired_person_id:
            self._closest_person = msg

    def _lookat_person(self):
        while not rospy.is_shutdown() and self.enabled:
            # Don't bother doing anything if we are disabled
            if self._closest_person is None:
                continue

            # Check if the closest person's look location is within bounds. If
            # not send a look command
            if self._last_look_pose is None or np.sqrt(
                (self._closest_person.pose.position.x - self._last_look_pose.position.x) ** 2
                + (self._closest_person.pose.position.y - self._last_look_pose.position.y) ** 2
                + (self._closest_person.pose.position.z - self._last_look_pose.position.z) ** 2
            ) >= LookAtClosestPersonAction.POSITION_CHANGE_HEAD_FOLLOW_THRESHOLD:
                self._last_look_pose = self._closest_person.pose
                self._look_action(pose={
                    'x': self._last_look_pose.position.x,
                    'y': self._last_look_pose.position.y,
                    'z': self._last_look_pose.position.z,
                    'frame': self._closest_person.header.frame_id,
                })
