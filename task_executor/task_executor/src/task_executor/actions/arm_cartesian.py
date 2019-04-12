#!/usr/bin/env python
# A cartesian arm action in a task plan. It converts velocity control to
# position control

from __future__ import print_function, division

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import TwistStamped


class ArmCartesianAction(AbstractStep):
    """
    A cartesian arm via position control. First perform linear motions, then the
    angular ones. Turns out that the cartesian arm controller is not available
    in simulation.
    """

    ARM_CARTESIAN_TOPIC = '/arm_controller/cartesian_twist/command'
    ARM_VEL = 0.7
    VEL_PUBLISH_FREQUENCY = 30.0

    def init(self, name):
        self.name = name
        self._cartesian_pub = rospy.Publisher(ArmCartesianAction.ARM_CARTESIAN_TOPIC, TwistStamped, queue_size=10)
        self._vel_publish_duration = rospy.Duration(1 / ArmCartesianAction.VEL_PUBLISH_FREQUENCY)
        self._stopped = False

    def run(self, linear_amount=[0.0, 0.0, 0.0], angular_amount=[0.0, 0.0, 0.0]):
        assert len(linear_amount) == len(angular_amount) == 3, \
            "Unexpected movement amounts: {}, {}".format(linear_amount, angular_amount)
        rospy.loginfo("Action {}: Linear by amount {}, Angular by amount {}"
                      .format(self.name, linear_amount, angular_amount))
        self._stopped = False

        # Figure out the duration to publish for based on the distance we wish
        # to travel
        desired_linear_duration = [amt / ArmCartesianAction.ARM_VEL for amt in linear_amount]
        desired_angular_duration = [amt / ArmCartesianAction.ARM_VEL for amt in angular_amount]

        # First do a linear move if one exists
        if sum([abs(x) for x in desired_linear_duration]) > 0 and not self._stopped:
            for variables in self._publish_velocity(desired_linear_duration, True):
                yield variables

        # Then an angular move if it exists
        if sum([abs(x) for x in desired_angular_duration]) > 0 and not self._stopped:
            for variables in self._publish_velocity(desired_angular_duration, False):
                yield variables

        # Finally, return succeeded if we hadn't been stopped
        if not self._stopped:
            yield self.set_succeeded()

    def stop(self):
        self._stopped = True

    def _publish_velocity(self, durations, is_linear):
        # Naively just do X, Y, Z
        for idx, duration in enumerate(durations):
            msg = TwistStamped()
            msg.header.frame_id = "base_link"
            aspect = msg.twist.linear if is_linear else msg.twist.angular
            if abs(duration) > 0:
                if idx == 0:
                    aspect.x = (duration / abs(duration)) * ArmCartesianAction.ARM_VEL
                elif idx == 1:
                    aspect.y = (duration / abs(duration)) * ArmCartesianAction.ARM_VEL
                elif idx == 2:
                    aspect.z = (duration / abs(duration)) * ArmCartesianAction.ARM_VEL

                # Then actually send the message
                for variables in self._publish_velocity_msg(msg, abs(duration)):
                    yield variables

            # Check if we've been stopped. If so, then break
            if self._stopped:
                break

    def _publish_velocity_msg(self, msg, duration):
        duration = rospy.Duration(duration)
        start_time = rospy.Time.now()
        last_publish_time = rospy.Time(0)
        while rospy.Time.now() < start_time + duration:
            # Publish if it's time to publish a move message
            if rospy.Time.now() > last_publish_time + self._vel_publish_duration:
                msg.header.stamp = rospy.Time.now()
                self._cartesian_pub.publish(msg)
                self.notify_topic_message(ArmCartesianAction.ARM_CARTESIAN_TOPIC, msg)
                last_publish_time = msg.header.stamp

            # Check to see that we haven't been preempted
            if self._stopped:
                yield self.set_preempted(
                    action=self.name,
                    status=GoalStatus.PREEMPTED,
                    goal=duration
                )
                raise StopIteration()

            # Otherwise yield control while we spin
            yield self.set_running()
            rospy.sleep(0.01)
