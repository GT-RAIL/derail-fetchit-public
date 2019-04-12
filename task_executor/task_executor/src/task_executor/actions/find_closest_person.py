#!/usr/bin/env python
# The find-closest-person action in a task plan

import rospy

from task_executor.abstract_step import AbstractStep
from sound_interface import SoundClient

from rail_people_detection_msgs.msg import Person, DetectionContext
from task_execution_msgs.msg import Bounds

from .look import LookAction
from .beep import BeepAction


class FindClosestPersonAction(AbstractStep):

    CLOSEST_PERSON_TOPIC = "/rail_people_detector/closest_person"
    EXPECTED_PERSON_ROI = Bounds(xmin=0.4, xmax=3.6, ymin=-2.4, ymax=2.4, zmin=1.4, zmax=2.0)
    PERSON_SEARCH_STEP_SIZES = { 'x': 0.8, 'y': 1.2, 'z': 0.3 }
    PERSON_SEARCH_WAIT_TIME = 1.5

    def init(self, name):
        self.name = name

        # Person detections
        self._selected_person = None            # This is person that we select
        self._last_closest_person = None        # These are the closest people
        self._select_closest_person = False     # Flag to trigger the save of a person
        self._closest_person_sub = rospy.Subscriber(
            FindClosestPersonAction.CLOSEST_PERSON_TOPIC,
            Person,
            self._on_closest_person
        )

        # Define the region of interest where we expect to find a person
        self._roi = FindClosestPersonAction.EXPECTED_PERSON_ROI
        self._step_sizes = FindClosestPersonAction.PERSON_SEARCH_STEP_SIZES

        self._search_wait_time = FindClosestPersonAction.PERSON_SEARCH_WAIT_TIME

        # Child actions
        self._look_action = LookAction()
        self._beep_action = BeepAction()

        # Set a stop flag
        self._stopped = False

        # Initialize the actions
        self._look_action.init('look_find_people')
        self._beep_action.init('beep_find_people')

    def run(self, timeout=0.0):
        # A timeout of 0 implies infinite
        rospy.loginfo("Action {}: Finding person within time {}s"
                      .format(self.name, timeout))

        # Set the flags for finding and waiting for a person
        self._stopped = False
        self._select_closest_person = True
        self._selected_person = None
        search_start_time = rospy.Time.now()

        while self._select_closest_person:

            z = self._roi.zmin
            while z <= self._roi.zmax and self._select_closest_person:

                x = self._roi.xmin
                while x <= self._roi.xmax and self._select_closest_person:

                    y = self._roi.ymin
                    while y <= self._roi.ymax and self._select_closest_person:
                        location = {'x': x, 'y': y, 'z': z, 'frame': 'base_link'}
                        self._look_action(pose=location)

                        # Wait for a detection, but yield control while waiting
                        start_time = rospy.Time.now()
                        while rospy.Time.now() <= start_time + rospy.Duration(self._search_wait_time):
                            if self._stopped:
                                yield self.set_preempted(
                                    action=self.name,
                                    goal=timeout,
                                    search_duration=(rospy.Time.now() - search_start_time).to_sec()
                                )
                                raise StopIteration()

                            if timeout > 0 and rospy.Time.now() - search_start_time > rospy.Duration(timeout):
                                yield self.set_aborted(
                                    action=self.name,
                                    goal=timeout,
                                    search_duration=(rospy.Time.now() - search_start_time).to_sec()
                                )
                                raise StopIteration()

                            yield self.set_running()

                        y += self._step_sizes['y']

                    x += self._step_sizes['x']
                    if not (z + self._step_sizes['z'] > self._roi.zmax and x > self._roi.xmax) \
                            and self._select_closest_person:
                        self._beep_action(beep=SoundClient.BEEP_UNSURE, async=True)

                z += self._step_sizes['z']

            if self._select_closest_person:
                self._beep_action(beep=SoundClient.BEEP_CONCERNED, async=True)

        # We have found our person. Yield them
        yield self.set_succeeded(person=self._selected_person)

    def stop(self):
        self._look_action.stop()
        self._beep_action.stop()
        self._stopped = True

    def _on_closest_person(self, msg):
        self._last_closest_person = msg

        # If we don't need to save the latest person as our selected target,
        # exit this callback
        if not self._select_closest_person:
            return

        # Choose this person only if they have a face
        if msg.detection_context.pose_source == DetectionContext.POSE_FROM_FACE:
            self._selected_person = self._last_closest_person
            self._select_closest_person = False
            self.notify_topic_message(FindClosestPersonAction.CLOSEST_PERSON_TOPIC, self._last_closest_person)
