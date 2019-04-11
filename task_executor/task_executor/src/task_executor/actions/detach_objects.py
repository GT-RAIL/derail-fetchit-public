#!/usr/bin/env python
# Try to detach all objects from the robot

from __future__ import print_function, division

import sys

import rospy
import moveit_commander

from task_executor.abstract_step import AbstractStep


# The action definition

class DetachObjectsAction(AbstractStep):

    ARM_GROUP_NAME = "arm"
    PLANNING_SCENE_TOPIC = "/planning_scene"
    ATTACHED_OBJECT_TOPIC = "/attached_collision_object"

    def init(self, name):
        self.name = name

        # Initialize the connection to MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        self._move_group = moveit_commander.MoveGroupCommander(DetachObjectsAction.ARM_GROUP_NAME)
        self._scene = moveit_commander.PlanningSceneInterface()

    def run(self):
        rospy.loginfo("Action {}: Detaching objects from planner".format(self.name))

        # First detach all objects from the arm
        result = self._move_group.detach_object()
        self.notify_topic_published(DetachObjectsAction.ATTACHED_OBJECT_TOPIC, None)
        rospy.sleep(0.5)

        # Then remove the objects from the scene
        self._scene.remove_world_object()
        self.notify_topic_published(DetachObjectsAction.PLANNING_SCENE_TOPIC, None)
        rospy.sleep(0.5)

        # Finally send a result
        if result:
            yield self.set_succeeded()
        else:
            yield self.set_aborted(
                action=self.name,
                result=result
            )

    def stop(self):
        # Cannot stop this action
        pass
