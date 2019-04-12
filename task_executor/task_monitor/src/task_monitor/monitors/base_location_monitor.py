#!/usr/bin/env python
# Base location monitor

from __future__ import print_function, division

import numpy as np

import rospy
import tf

from task_execution_msgs.msg import BeliefKeys, Waypoint
from task_execution_msgs.srv import GetWaypoints

from task_monitor.monitoring import AbstractBeliefMonitor


# The class definition

class BaseLocationMonitor(AbstractBeliefMonitor):
    """
    Monitor the location of the base in the map and update the robot's belief of
    the base location accordingly.
    """

    BASE_FRAME = "/base_link"           # Robot position
    DESIRED_COMPARISON_FRAME = "/map"   # Reference locations will be stored in this frame
    MONITORING_LOOP_RATE = 1            # Hz
    LOCATION_RADIUS = 0.25              # Should be within a 25 cm radius of the location

    TRANSFORM_WAIT_DURATION = rospy.Duration(10.0)

    WAYPOINTS_SERVICE_NAME = "/database/waypoints"
    PICKUP_1_LOCATION_NAMES = ["table1",]
    DOOR_1_LOCATION_NAMES = [
        "door1_out_facing_in",
        "door1_out_facing_out",
        "door1_in_facing_in",
        "door1_in_facing_out",
    ]
    DROPOFF_LOCATION_NAMES = ["shelf",]

    def __init__(self):
        super(BaseLocationMonitor, self).__init__()

        # Create the tf listener
        self._listener = tf.TransformListener()

        # Connect to the database service and get the waypoint positions
        get_waypoints_srv = rospy.ServiceProxy(BaseLocationMonitor.WAYPOINTS_SERVICE_NAME, GetWaypoints)
        rospy.loginfo("Connecting to database services...")
        get_waypoints_srv.wait_for_service()
        rospy.loginfo("...database services connected")
        belief_to_locations_map = {
            BeliefKeys.ROBOT_AT_PICKUP_1: BaseLocationMonitor.PICKUP_1_LOCATION_NAMES,
            BeliefKeys.ROBOT_AT_DOOR_1: BaseLocationMonitor.DOOR_1_LOCATION_NAMES,
            BeliefKeys.ROBOT_AT_DROPOFF: BaseLocationMonitor.DROPOFF_LOCATION_NAMES,
        }

        # Create the dictionary of waypoint positions to check against
        self._location_belief_checks = {
            BeliefKeys.ROBOT_AT_PICKUP_1: [],
            BeliefKeys.ROBOT_AT_DOOR_1: [],
            BeliefKeys.ROBOT_AT_DROPOFF: [],
        }
        for location_belief in self._location_belief_checks.iterkeys():
            for location_name in belief_to_locations_map[location_belief]:
                # Get the waypoints and add each to the list
                waypoints = get_waypoints_srv(location_name).waypoints
                for waypoint in waypoints:
                    # We do want to die on an exception, so don't catch anything
                    # here
                    self._listener.waitForTransform(
                        BaseLocationMonitor.DESIRED_COMPARISON_FRAME,
                        waypoint.frame,
                        rospy.Time(),
                        BaseLocationMonitor.TRANSFORM_WAIT_DURATION
                    )
                    (trans, _) = self._listener.lookupTransform(
                        BaseLocationMonitor.DESIRED_COMPARISON_FRAME,
                        waypoint.frame,
                        rospy.Time(0)
                    )
                    self._location_belief_checks[location_belief].append(np.array([
                        waypoint.x + trans[0],
                        waypoint.y + trans[1],
                    ]))

        # Finally, start the monitor on a timer
        self._monitor_timer = rospy.Timer(
            rospy.Duration(1 / BaseLocationMonitor.MONITORING_LOOP_RATE),
            self._monitor_func,
            oneshot=False
        )

    def _monitor_func(self, evt):
        # Get the porition of the base
        try:
            (trans, _) = self._listener.lookupTransform(
                BaseLocationMonitor.DESIRED_COMPARISON_FRAME,
                BaseLocationMonitor.BASE_FRAME,
                rospy.Time(0)
            )
            trans = np.array(trans)
        except (tf.LookupException, tf.ExtrapolationException):
            return None

        # Then update the belief based on whether the position is within the
        # tolerance
        beliefs = {
            belief: np.any([
                np.linalg.norm(pos - trans[:2]) <= BaseLocationMonitor.LOCATION_RADIUS
                for pos in positions
            ])
            for belief, positions in self._location_belief_checks.iteritems()
        }

        # Send the updated values and return the messages that were sent
        return self.update_beliefs(beliefs)


# When running the monitor in standalone mode
if __name__ == '__main__':
    rospy.init_node('base_location_monitor')
    monitor = BaseLocationMonitor()
    rospy.spin()
