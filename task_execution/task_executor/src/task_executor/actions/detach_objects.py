#!/usr/bin/env python
# Try to detach all objects from the robot

from __future__ import print_function, division

import rospy

from task_executor.abstract_step import AbstractStep

from std_srvs.srv import Empty


# The action definition

class DetachObjectsAction(AbstractStep):

    DETACH_FROM_ARM_SERVICE = '/collision_scene_manager/detach_objects'
    DETACH_FROM_BASE_SERVICE = '/collision_scene_manager/detach_all_from_base'

    def init(self, name):
        self.name = name

        # The services to detach objects through the Collision Scene Manager
        self._detach_arm_srv = rospy.ServiceProxy(
            DetachObjectsAction.DETACH_FROM_ARM_SERVICE,
            Empty
        )
        self._detach_base_srv = rospy.ServiceProxy(
            DetachObjectsAction.DETACH_FROM_BASE_SERVICE,
            Empty
        )

        # Connect to the services
        rospy.loginfo("Connecting to collision_scene_manager...")
        self._detach_arm_srv.wait_for_service()
        self._detach_base_srv.wait_for_service()
        rospy.loginfo("...collision_scene_manager connected")

    def run(self, detach_arm=False, detach_base=False):
        rospy.loginfo("Action {}: Detaching from arm({}) and base({})".format(
            self.name, detach_arm, detach_base
        ))

        # First detach all objects from the arm
        if detach_arm:
            self._detach_arm_srv()
            self.notify_service_called(DetachObjectsAction.DETACH_FROM_ARM_SERVICE)
            yield self.set_running()

        # Then detach all objects from the base
        if detach_base:
            self._detach_base_srv()
            self.notify_service_called(DetachObjectsAction.DETACH_FROM_BASE_SERVICE)
            yield self.set_running()

        # Finally yield a success unless there was a service exception
        yield self.set_succeeded()

    def stop(self):
        # Cannot stop this action
        pass
