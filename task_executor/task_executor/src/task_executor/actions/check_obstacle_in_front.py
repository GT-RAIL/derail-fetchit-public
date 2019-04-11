#!/usr/bin/env python
# Check to see if there's an obstacle in a narrow window ahead of us

from __future__ import print_function, division

import numpy as np

import rospy

from task_executor.abstract_step import AbstractStep

from sensor_msgs.msg import LaserScan
from task_execution_msgs.msg import BeliefKeys


class CheckObstacleInFrontAction(AbstractStep):
    """
    Use the laser scan (for now) to check if the path ahead of us is blocked
    """

    LASER_SCAN_TOPIC = "/base_scan"
    DISTANCE_AHEAD_TO_CHECK = 0.86  # Check 86cm in front; we travel ~1.72m through a door
    WIDTH_TO_CHECK = 0.1  # Check a window 20cm ahead for the presence/absence of a door

    def init(self, name):
        self.name = name

        # Set a stopped flag
        self._stopped = False

    def run(self, belief, binarize=False, abort_on_true=False, update_negation=False):
        # binarize - belief should be updated as a binary, not a float
        # abort_on_true - set an abort if there is an obstacle, else return the value
        # update_negation - when updating the belief, update the negated value
        belief = getattr(BeliefKeys, belief, belief)
        rospy.loginfo("Action {}: Checking for an obstacle to update belief {}"
                      .format(self.name, belief))

        # Set the stopped flag and wait for a message
        self._stopped = False
        laser_scan = None
        while laser_scan is None:
            try:
                laser_scan = rospy.wait_for_message(CheckObstacleInFrontAction.LASER_SCAN_TOPIC, LaserScan, 0.1)
                self.notify_topic_message(CheckObstacleInFrontAction.LASER_SCAN_TOPIC, laser_scan)
            except rospy.ROSException as e:
                pass

            if self._stopped:
                yield self.set_preempted(action=self.name)
                raise StopIteration()

            yield self.set_running()

        # Calculate if there is something within the bounds
        ranges = np.array(laser_scan.ranges)
        angle_to_check = np.arctan(CheckObstacleInFrontAction.WIDTH_TO_CHECK
                                   / CheckObstacleInFrontAction.DISTANCE_AHEAD_TO_CHECK)
        num_indices = int(np.ceil(angle_to_check / laser_scan.angle_increment))
        mid_index = len(ranges) // 2
        ranges_too_close = \
            ranges[mid_index-num_indices:mid_index+num_indices] < CheckObstacleInFrontAction.DISTANCE_AHEAD_TO_CHECK
        if binarize:
            value = np.any(ranges_too_close)
        else:
            value = np.sum(ranges_too_close) / len(ranges_too_close)

        # Update the belief
        self.update_beliefs({
            belief: ((not value) if binarize else (1 - value)) if update_negation else value
        })

        # Return according to the specification
        if value and abort_on_true:
            yield self.set_aborted(action=self.name, obstacle_in_front=value)
        else:  # not value or not abort_on_true
            yield self.set_succeeded(obstacle_in_front=value)

    def stop(self):
        self._stopped = True
