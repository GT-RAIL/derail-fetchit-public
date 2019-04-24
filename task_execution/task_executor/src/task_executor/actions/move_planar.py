#!/usr/bin/env python
# A planar move action in a task plan. It converts velocity control to position
# control

from __future__ import print_function, division

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist


class MovePlanarAction(AbstractStep):
    """
    A planar move via position control. Do angular move first and then linear.
    This movement is not precise and is instead meant for the sorts of coarse
    directional movements that can sometimes be useful for recovery.
    """

    BASE_VEL_TOPIC = '/cmd_vel'
    BASE_LINEAR_VEL = 0.5
    BASE_ANGULAR_VEL = 0.7
    VEL_PUBLISH_FREQUENCY = 20.0

    def init(self, name):
        self.name = name
        self._cmd_vel_pub = rospy.Publisher(MovePlanarAction.BASE_VEL_TOPIC, Twist, queue_size=10)
        self._vel_publish_duration = rospy.Duration(1 / MovePlanarAction.VEL_PUBLISH_FREQUENCY)
        self._stopped = False

    def run(self, linear_amount=0.0, angular_amount=0.0):
        """
        The run function for this step

        Args:
            linear_amount (float) : the amount in meters to move forward (+ve)
                or backward (-ve)
            angular_amount (float) : the amount in radians to spin left (+ve)
                or right (-ve)

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo("Action {}: Linear by amount {}, Angular by amount {}"
                      .format(self.name, linear_amount, angular_amount))
        self._stopped = False

        # Figure out the duration to publish for based on the distance we wish
        # to travel
        desired_linear_duration = abs(linear_amount / MovePlanarAction.BASE_LINEAR_VEL)
        desired_angular_duration = abs(angular_amount / MovePlanarAction.BASE_ANGULAR_VEL)

        # Create the message
        msg = Twist()

        # First an angular move if it exists
        if desired_angular_duration > 0 and not self._stopped:
            msg.linear.x = 0.0
            msg.angular.z = (angular_amount / abs(angular_amount)) * MovePlanarAction.BASE_ANGULAR_VEL
            for variables in self._publish_velocity_msg(msg, desired_angular_duration):
                yield variables

        # Then do a linear move if one exists
        if desired_linear_duration > 0 and not self._stopped:
            msg.angular.z = 0.0
            msg.linear.x = (linear_amount / abs(linear_amount)) * MovePlanarAction.BASE_LINEAR_VEL
            for variables in self._publish_velocity_msg(msg, desired_linear_duration):
                yield variables

        # Finally, return succeeded if we hadn't been stopped
        if not self._stopped:
            yield self.set_succeeded()

    def stop(self):
        self._stopped = True

    def _publish_velocity_msg(self, msg, duration):
        duration = rospy.Duration(duration)
        start_time = rospy.Time.now()
        last_publish_time = rospy.Time(0)
        while rospy.Time.now() < start_time + duration:
            # Publish if it's time to publish a move message
            if rospy.Time.now() > last_publish_time + self._vel_publish_duration:
                self._cmd_vel_pub.publish(msg)
                self.notify_topic_message(MovePlanarAction.BASE_VEL_TOPIC, msg)
                last_publish_time = rospy.Time.now()

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
