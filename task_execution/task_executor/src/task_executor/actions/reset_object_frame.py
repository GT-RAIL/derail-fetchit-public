#!/usr/bin/env python
# The action to reset object_frame from in-hand localization

from __future__ import print_function, division

import rospy

from task_executor.abstract_step import AbstractStep

from std_srvs.srv import Empty


class ResetObjectFrameAction(AbstractStep):
    """
    If you want to reset the object frame (for example, to skip in-hand
    localization), call this service
    :const:`RESET_FRAME_SERVICE_NAME` to reset the object frame.
    """

    RESET_FRAME_SERVICE_NAME = '/in_hand_localizer/reset_object_frame'

    def init(self, name):
        self.name = name

        # The bin detection interface
        self._reset_object_frame_srv = rospy.ServiceProxy(
            ResetObjectFrameAction.RESET_FRAME_SERVICE_NAME,
            Empty
        )

        # Set a stop flag
        self._stopped = False

        # Wait for the connection to in-hand localization
        rospy.loginfo("Connecting to reset object frame service...")
        self._reset_object_frame_srv.wait_for_service()
        rospy.loginfo("...reset object frame service connected")

    def run(self):
        """
        The run function for this step

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo(
            "Action {}: Resetting the in-hand object frame.".format(
                self.name,
            )
        )
        self._stopped = False

        # Reset the object frame
        success = self._reset_object_frame_srv()

        yield self.set_running()  # Check the status of the server

        if self._stopped:
            yield self.set_preempted(
                action=self.name,
                srv=self._reset_object_frame_srv.resolved_name,
            )
        elif not success:
            rospy.logerr("Action {}: Could not reset object_frame".format(self.name))
            yield self.set_aborted(
                action=self.name,
                srv=self._reset_object_frame_srv.resolved_name,
            )
        else:
            yield self.set_succeeded()

    def stop(self):
        self._stopped = True
