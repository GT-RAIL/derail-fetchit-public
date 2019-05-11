#!/usr/bin/python
# Move back to the origin. You should be facing in roughly the same direction
# you were before the move backward

from __future__ import print_function, division

import rospy
import tf2_ros

from task_executor.abstract_step import AbstractStep

from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import euler_from_quaternion

from .reposition import RepositionAction


# The class definition

class MoveBackwardAction(AbstractStep):
    """
    Used to move backward from the current pose by the specified amount. Uses
    the :mod:`task_executor.actions.reposition` action internally
    """

    MAP_FRAME = "map"
    BASE_FRAME = "base_link"

    def init(self, name):
        self.name = name

        # Create the tf listener
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # Initialize the reposition action
        self._reposition = RepositionAction()

        # Initialize the internal action
        self._reposition.init('reposition_move_backward')

    def run(self, amount):
        """
        The run function for this step

        Args:
            amount (float) : the amount in meters to move backward (+ve) by

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo("Action {}: Moving backward by {}".format(self.name, amount))

        # First set the amount we want to move backward by as a PoseStamped
        desired_pose = PoseStamped()
        desired_pose.header.frame_id = MoveBackwardAction.BASE_FRAME
        desired_pose.pose.position.x = -amount

        # Then get that pose in the map frame
        try:
            transform = self._tf_buffer.lookup_transform(
                MoveBackwardAction.MAP_FRAME,
                MoveBackwardAction.BASE_FRAME,
                rospy.Time(0),
                rospy.Duration(1.0)
            )

            desired_pose = do_transform_pose(desired_pose, transform)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            yield self.set_aborted(
                action=self.name,
                goal=amount,
                exception=e
            )
            raise StopIteration()

        # Convert the pose back to a format that the reposition action
        # understands
        rotation = euler_from_quaternion([
            desired_pose.pose.orientation.x,
            desired_pose.pose.orientation.y,
            desired_pose.pose.orientation.z,
            desired_pose.pose.orientation.w,
        ])
        location = {
            "x": desired_pose.pose.position.x,
            "y": desired_pose.pose.position.y,
            "theta": rotation[2],
            "frame": MoveBackwardAction.MAP_FRAME,
        }

        # Send the goal to reposition action and wait until it succeeds or fails
        for variables in self._reposition.run(location=location):
            yield self.set_running(**variables)

        # Then exit based on how reposition exited
        if self._reposition.is_preempted():
            yield self.set_preempted(
                action=self.name,
                goal=amount,
                context=variables
            )
        elif self._reposition.is_aborted():
            yield self.set_aborted(
                action=self.name,
                goal=amount,
                context=variables
            )
        else:  # Succeeded
            yield self.set_suceeded()

    def stop(self):
        self._reposition.stop()
