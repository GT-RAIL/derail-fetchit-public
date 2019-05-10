#!/usr/bin/env python
# The move action in a task plan

from __future__ import print_function, division

from math import sin, cos

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from fetchit_mapping.msg import NavigationAction, NavigationGoal
from actionlib_msgs.msg import GoalStatus
from task_execution_msgs.msg import Waypoint, BeliefKeys
from task_execution_msgs.srv import GetWaypoints, GetSemanticLocations


class MoveAction(AbstractStep):
    """
    Move to a location, or a series of locations in the map. Requires a
    localized robot
    """

    MOVE_ACTION_SERVER = "/navigation"
    WAYPOINTS_SERVICE_NAME = "/database/waypoints"
    SEMANTIC_LOCATIONS_SERVICE_NAME = "/database/semantic_locations"
    BELIEF_KEYS = [x for x in dir(BeliefKeys) if x.isupper()]

    def init(self, name):
        self.name = name
        self._navigation_client = actionlib.SimpleActionClient(MoveAction.MOVE_ACTION_SERVER, NavigationAction)
        self._get_waypoints_srv = rospy.ServiceProxy(MoveAction.WAYPOINTS_SERVICE_NAME, GetWaypoints)
        self._get_semantic_locations_srv = rospy.ServiceProxy(
            MoveAction.SEMANTIC_LOCATIONS_SERVICE_NAME,
            GetSemanticLocations
        )

        rospy.loginfo("Connecting to navigation...")
        self._navigation_client.wait_for_server()
        rospy.loginfo("...navigation connected")

        rospy.loginfo("Connecting to database services...")
        self._get_waypoints_srv.wait_for_service()
        rospy.loginfo("...database services connected")

        rospy.loginfo("connecting to database services...")
        self._get_semantic_locations_srv.wait_for_service()
        rospy.loginfo("...database services connected")

        # Get all possible semantic locations
        self.semantic_locations = self._get_semantic_locations_srv().locations

    def run(self, location, update_belief=True):
        """
        The run function for this step

        Args:
            location (str, list, tuple, dict) :
                The location to move to. If the type is:

                * str. Then if the string starts with
                    * `locations`, assume the rest of the string specifies the \
                        ``tf`` frame of the waypoint and therefore move to the \
                        pose ``[0, 0, 0]`` w.r.t that frame
                    * `waypoints`, get a list of ``task_execution_msgs/Waypoint`` \
                        poses from :const:`WAYPOINTS_SERVICE_NAME`; visit the \
                        waypoints in order
                * dict. Then if the keys of the dict are
                    * `x, y, theta, frame`, visit the waypoint defined by the dict
                * list, tuple. Then if the list is of
                    * `dicts of the previous case`, visit the waypoints in the \
                        list or tuple in order

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        # Parse out the waypoints
        coords, semantic_location = self._parse_location(location)
        update_belief = update_belief & (semantic_location is not None)
        if coords is None:
            rospy.logerr("Action {}: FAIL. Unknown Format: {}".format(self.name, location))
            raise KeyError(self.name, "Unknown Format", location)

        rospy.logdebug("Action {}: Moving to location(s): {}".format(self.name, coords))

        status = GoalStatus.LOST
        for coord_num, coord in enumerate(coords):
            rospy.loginfo("Action {}: Going to {}/{}. Coordinate: {{ {} }}"
                          .format(self.name, coord_num + 1, len(coords), str(coord).replace("\n", ", ")))

            # Create and send the goal
            goal = NavigationGoal()
            goal.goal.pose.position.x = coord.x
            goal.goal.pose.position.y = coord.y
            goal.goal.pose.orientation.z = sin(coord.theta/2.0)
            goal.goal.pose.orientation.w = cos(coord.theta/2.0)
            goal.goal.header.frame_id = coord.frame
            goal.goal.header.stamp = rospy.Time.now()
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
            # update task belief if move is successful
            if update_belief:
                self._update_location_belief(semantic_location)
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
        semantic_location = None
        if isinstance(location, str):
            db_name, location = location.split('.', 1)
            if db_name == 'waypoints':
                coords = self._get_waypoints_srv(location).waypoints
                self.notify_service_called(MoveAction.WAYPOINTS_SERVICE_NAME)
                semantic_location = location
            elif db_name == 'locations':
                # These are predefined tf frames
                coords = [Waypoint(frame=location)]
        elif isinstance(location, dict):
            coords = [Waypoint(**location),]
        elif isinstance(location, (list, tuple,)):
            coords = [Waypoint(**x) for x in location]

        return coords, semantic_location

    def _update_location_belief(self, semantic_location):
        beliefs = {}
        # Set task belief about goal location to true
        belief_key = ("task_at_" + semantic_location).upper()
        if belief_key in MoveAction.BELIEF_KEYS:
            beliefs[getattr(BeliefKeys, belief_key)] = True
        # Set task beliefs about all other locations to false
        for location in self.semantic_locations:
            bk = ("task_at_" + location).upper()
            if bk == belief_key:
                continue
            if bk in MoveAction.BELIEF_KEYS:
                beliefs[getattr(BeliefKeys, bk)] = False

        rospy.loginfo("Action {}: update task belief: {}".format(self.name, beliefs))
        self.update_beliefs(beliefs)

        return
