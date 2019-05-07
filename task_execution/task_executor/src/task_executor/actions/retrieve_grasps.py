#!/usr/bin/env python
# Retrieves hard-coded grasps for desired objects

from __future__ import print_function, division

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from fetch_grasp_suggestion.srv import RetrieveGrasps
from manipulation_actions.msg import ChallengeObject


class RetrieveGraspsAction(AbstractStep):
    """
    Given a ``manipulation_actions/ChallengeObject`` object returned from
    ``rail_segmentation``, return the hard coded grasps on it. We return at most
    :const:`MAX_GRASPS` number of grasps.
    """

    RETRIEVE_GRASPS_SERVICE_NAME = "/grasp_retriever/retrieve_grasps"
    MAX_GRASPS = 100
    CHALLENGE_OBJ_NAME_LOOKUP = {
        getattr(ChallengeObject, x): x for x in dir(ChallengeObject) if x.isupper()
    }

    def init(self, name):
        self.name = name

        # The Grasp calculation interface
        self._retrieve_grasps_srv = rospy.ServiceProxy(
            RetrieveGraspsAction.RETRIEVE_GRASPS_SERVICE_NAME,
            RetrieveGrasps
        )

        # Set the max number of grasps to try. This can be a param lookup
        self._max_grasps = RetrieveGraspsAction.MAX_GRASPS

        # Set a stop flag
        self._stopped = False

        # Wait for a connection to retrieve_grasps
        rospy.loginfo("Connecting to retrieve_grasps...")
        self._retrieve_grasps_srv.wait_for_service()
        rospy.loginfo("...retrieve_grasps connected")

    def run(self, segmented_obj, object_type):
        """
        The run function for this step

        Args:
            segmented_obj (rail_manipulation_msgs/SegmentedObject) :
                a segmented object returned from ``rail_segmentation``
            object_type (manipulation_actions/ChallengeObject) : the type of
                challenge object that we wish to retrieve grasps for

        Yields:
            grasps (list of geometry_msgs/Pose) : a list of at most \
                :const:`MAX_GRASPS` grasps

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        if isinstance(object_type, str):
            object_type = getattr(ChallengeObject, object_type.upper())

        rospy.loginfo("Action {}: Retrieving grasps for object({}) at: {}".format(
            self.name,
            RetrieveGraspsAction.CHALLENGE_OBJ_NAME_LOOKUP.get(object_type),
            str(segmented_obj.center).replace("\n", ", ")
        ))

        self._stopped = False

        # Given the segmentation and the objects, now ask for grasps
        grasps = self._retrieve_grasps_srv(
            object=segmented_obj,
            type=ChallengeObject(object_type)
        ).grasp_list
        self.notify_service_called(RetrieveGraspsAction.RETRIEVE_GRASPS_SERVICE_NAME)
        yield self.set_running()
        if len(grasps.poses) == 0:
            yield self.set_aborted(
                action=self.name,
                goal=segmented_obj,
                srv=self._retrieve_grasps_srv.resolved_name,
                num_grasps=len(grasps.poses)
            )
        elif self._stopped:
            yield self.set_preempted(
                action=self.name,
                goal=segmented_obj,
                srv=self._retrieve_grasps_srv.resolved_name,
                num_grasps=len(grasps.poses)
            )
        else:
            yield self.set_running()

        # Gratuitous self.running() above
        grasps.poses = grasps.poses[:self._max_grasps]
        yield self.set_succeeded(grasps=grasps)

    def stop(self):
        self._stopped = True
