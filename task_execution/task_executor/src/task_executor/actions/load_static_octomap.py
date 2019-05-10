#!/usr/bin/env python
# The action to reload the static octomap into MoveIt!

from __future__ import print_function, division

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from moveit_msgs.srv import LoadMap
from std_srvs.srv import Empty


class LoadStaticOctomapAction(AbstractStep):
    """
    If you want to reset the Moveit Octomap for planning, call this service
    :const:`LOAD_MAP_SERVICE_NAME` to load the static octomap. Requires that
    the :const:`OCTOMAP_RELOAD_PATH` parameter is set appropriately
    """

    LOAD_MAP_SERVICE_NAME = '/move_group/load_map'
    CLEAR_OBJECTS_SERVICE_NAME = '/collision_scene_manager/clear_unattached_objects'
    OCTOMAP_RELOAD_PATH = '/octomap_reload_path'

    def init(self, name):
        self.name = name

        self._filename = rospy.get_param(LoadStaticOctomapAction.OCTOMAP_RELOAD_PATH)

        # The bin detection interface
        self._load_map_srv = rospy.ServiceProxy(
            LoadStaticOctomapAction.LOAD_MAP_SERVICE_NAME,
            LoadMap
        )

        self._clear_objects_srv = rospy.ServiceProxy(
            LoadStaticOctomapAction.CLEAR_OBJECTS_SERVICE_NAME,
            Empty
        )

        # Set a stop flag
        self._stopped = False

        # Wait for the connection to the bin detector
        rospy.loginfo("Connecting to /move_group/load_map...")
        self._load_map_srv.wait_for_service()
        rospy.loginfo(".../move_group/load_map connected")

    def run(self):
        """
        The run function for this step

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo(
            "Action {}: Loading static octomap to moveit. octomap file path: {}".format(
                self.name,
                self._filename,
            )
        )
        self._stopped = False

        # Clear out collision objects from the collision scene manager
        self._clear_objects_srv()

        # Attempt to load the static octomap to moveit
        success = self._load_map_srv(self._filename).success
        self.notify_service_called(LoadStaticOctomapAction.LOAD_MAP_SERVICE_NAME)
        yield self.set_running()  # Check the status of the server

        if self._stopped:
            yield self.set_preempted(
                action=self.name,
                srv=self._load_map_srv.resolved_name,
            )
        elif not success:
            rospy.logerr("Action {}: Could not load the static octomap to moveit".format(self.name))
            yield self.set_aborted(
                action=self.name,
                srv=self._load_map_srv.resolved_name,
            )
        else:
            yield self.set_succeeded()

    def stop(self):
        self._stopped = True
