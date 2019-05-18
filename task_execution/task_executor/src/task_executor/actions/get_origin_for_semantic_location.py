#!/usr/bin/env python
# Get the origin for a particular move location

from __future__ import print_function, division

import rospy

from task_executor.abstract_step import AbstractStep

from task_execution_msgs.srv import GetSemanticLocations


class GetOriginForSemanticLocation(AbstractStep):
    """
    Given a semantic location, get the ``origin_for_`` location, or None if one
    does not exist
    """

    SEMANTIC_LOCATIONS_SERVICE_NAME = "/database/semantic_locations"
    ORIGIN_FOR_LOCATION_FORMAT = "origin_for_{}"

    def init(self, name):
        self.name = name
        self._get_semantic_locations_srv = rospy.ServiceProxy(
            GetOriginForSemanticLocation.SEMANTIC_LOCATIONS_SERVICE_NAME,
            GetSemanticLocations
        )

        # Set a stop flag
        self._stopped = False

        # Wait for a connection to the beliefs
        rospy.loginfo("Connecting to database services...")
        self._get_semantic_locations_srv.wait_for_service()
        rospy.loginfo("...database services connected")

        # Get all possible semantic locations
        self.origins = self._parse(self._get_semantic_locations_srv().locations)

    def run(self, location):
        """
        The run function for this step

        Args:
            location (str) : The waypoint to get the ``origin_for_`` location to

        Yields:
            origin_location (str or None)

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo("Action {}: Getting origin for {}".format(self.name, location))
        origin_location = None

        # Try to get the semantic origin only if the format is one we recognize
        if isinstance(location, str) and '.' in location:
            prefix, location_name = location.split('.', 1)
            if prefix == 'waypoints' and self.origins.get(location_name) is not None:
                origin_location = '{}.{}'.format(prefix, self.origins.get(location_name))

        # Sanity check that the requested keys
        yield self.set_succeeded(origin_location=origin_location)

    def stop(self):
        pass

    def _parse(self, locations):
        origins = {}
        locations = set(locations)
        for location in locations:
            if GetOriginForSemanticLocation.ORIGIN_FOR_LOCATION_FORMAT.format(location) in locations:
                origins[location] = GetOriginForSemanticLocation.ORIGIN_FOR_LOCATION_FORMAT.format(location)
        return origins
