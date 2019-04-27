#!/usr/bin/env python
# The action to detect bin poses and enter it into the tf tree

from __future__ import print_function, division

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from moveit_msgs.srv import LoadMap


class LoadStaticOctomapAction(AbstractStep):
    """
    If you want to set the Moveit Octomap for planning, call this service
    :const:`LOAD_MAP_SERVICE_NAME` to load the static octomap.
    """

    LOAD_MAP_SERVICE_NAME = '/move_group/load_map'

    def init(self, name):
        self.name = name

        self._filename = rospy.get_param('/octomap_reload_path')

        # The bin detection interface
        self._load_map_srv = rospy.ServiceProxy(
            LoadStaticOctomapAction.LOAD_MAP_SERVICE_NAME,
            LoadMap
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

        Args:
            filename (string) : absolute file path of the static octomap to load

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

        # Attempt to load the static octomap to moveit
        success = self._load_map_srv(self._filename)
        self.notify_service_called(LoadStaticOctomapAction.LOAD_MAP_SERVICE_NAME)
        yield self.set_running()  # Check the status of the server

        if self._stopped:
            yield self.set_preempted(
                action=self.name,
                srv=self._load_map_srv.resolved_name,
            )
        elif not success:
            rospy.logerror("Action {}: Could not load the static octomap to moveit".format(self.name))
            yield self.set_aborted(
                action=self.name,
                srv=self._load_map_srv.resolved_name,
            )
        else:
            yield self.set_succeeded()

    def stop(self):
        self._stopped = True
