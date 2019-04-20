#!/usr/bin/env python
# The move action in a task plan

from __future__ import print_function, division

from math import sin, cos

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from fetchit_mapping.msg import NavigationAction, NavigationGoal
from actionlib_msgs.msg import GoalStatus
from task_execution_msgs.msg import Waypoint
from task_execution_msgs.srv import GetWaypoints


class MoveAction(AbstractStep):
    """Move to a location"""

    MOVE_ACTION_SERVER = "/navigation"
    WAYPOINTS_SERVICE_NAME = "/database/waypoints"

    def init(self, name):
        self.name = name
        self._navigation_client = actionlib.SimpleActionClient(MoveAction.MOVE_ACTION_SERVER, NavigationAction)
        self._get_waypoints_srv = rospy.ServiceProxy(MoveAction.WAYPOINTS_SERVICE_NAME, GetWaypoints)

        rospy.loginfo("Connecting to navigation...")
        self._navigation_client.wait_for_server()
        rospy.loginfo("...navigation connected")

        rospy.loginfo("Connecting to database services...")
        self._get_waypoints_srv.wait_for_service()
        rospy.loginfo("...database services connected")

    def run(self, location):
        # Parse out the waypoints
        coords = self._parse_location(location)
        if coords is None:
            rospy.logerr("Action {}: FAIL. Unknown Format: {}".format(self.name, location))
            raise KeyError(self.name, "Unknown Format", location)

        rospy.logdebug("Action {}: Moving to location(s): {}".format(self.name, coords))

        status = GoalStatus.LOST
        for coord_num, coord in enumerate(coords):
            rospy.loginfo("Action {}: Going to {}/{}. Coordinate: {{ {} }}"
                          .format(self.name, coord_num + 1, len(coords), str(coord).replace("\n", ", ")))

            # Create and send the goal
            goal = MoveBaseGoal()
            goal.target_pose.pose.position.x = coord.x
            goal.target_pose.pose.position.y = coord.y
            goal.target_pose.pose.orientation.z = sin(coord.theta/2.0)
            goal.target_pose.pose.orientation.w = cos(coord.theta/2.0)
            goal.target_pose.header.frame_id = coord.frame
            goal.target_pose.header.stamp = rospy.Time.now()
            self._navigation_client.send_goal(goal)
            self.notify_action_send_goal(MoveAction.MOVE_ACTION_SERVER, goal)

            # Yield running while the move_client is executing
            while self._navigation_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
                yield self.set_running()

            # Check the status and stop executing if we didn't complete our goal
            status = self._navigation_client.get_state()
            self._navigation_client.wait_for_result()
            result = self._navigation_client.get_result()
            self.notify_action_recv_result(MoveAction.MOVE_ACTION_SERVER, status, result)

            if status != GoalStatus.SUCCEEDED:
                break

        # Yield based on how we exited
        if status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted(
                action=self.name,
                status=status,
                goal=goal,
                coord_num=coord_num,
                result=result
            )
        else:
            yield self.set_aborted(
                action=self.name,
                status=status,
                goal=goal,
                coord_num=coord_num,
                result=result
            )

    def stop(self):
        self._navigation_client.cancel_goal()
        self.notify_action_cancel(MoveAction.MOVE_ACTION_SERVER)

    def _parse_location(self, location):
        coords = None
        if isinstance(location, str):
            db_name, location = location.split('.', 1)
            if db_name == 'locations':
                coords = self._get_waypoints_srv(location).waypoints
                self.notify_service_called(MoveAction.WAYPOINTS_SERVICE_NAME)
        elif isinstance(location, dict):
            coords = [Waypoint(**location),]
        elif isinstance(location, (list, tuple,)):
            coords = [Waypoint(**x) for x in location]

        return coords
