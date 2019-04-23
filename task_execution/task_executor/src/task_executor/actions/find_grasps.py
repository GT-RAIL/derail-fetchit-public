#!/usr/bin/env python
# The find object action in a task plan

from __future__ import print_function, division

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from fetch_grasp_suggestion.srv import SuggestGrasps, PairwiseRank


class FindGraspsAction(AbstractStep):
    """
    Given an object returned from ``rail_segmentation``, calculate grasps on it.
    We return at most :const:`MAX_GRASPS` number of grasps.
    """

    SUGGEST_GRASPS_SERVICE_NAME = "/suggester/suggest_grasps"
    PAIRWISE_RANK_SERVICE_NAME = "/suggester/pairwise_rank"
    MAX_GRASPS = 10

    def init(self, name):
        self.name = name

        # The Grasp calculation interface
        self._suggest_grasps_srv = rospy.ServiceProxy(
            FindGraspsAction.SUGGEST_GRASPS_SERVICE_NAME,
            SuggestGrasps
        )
        self._grasps_rank_srv = rospy.ServiceProxy(
            FindGraspsAction.PAIRWISE_RANK_SERVICE_NAME,
            PairwiseRank
        )

        # Set the max number of grasps to try. This can be a param lookup
        self._max_grasps = FindGraspsAction.MAX_GRASPS

        # Set a stop flag
        self._stopped = False

        # Wait for a connection to suggest_grasps
        rospy.loginfo("Connecting to suggest_grasps...")
        self._suggest_grasps_srv.wait_for_service()
        self._grasps_rank_srv.wait_for_service()
        rospy.loginfo("...suggest_grasps connected")

    def run(self, segmented_obj):
        """
        The run function for this step

        Args:
            segmented_obj (rail_manipulation_msgs/SegmentedObject) :
                a segmented object returned from ``rail_segmentation``

        Yields:
            grasps (list of geometry_msgs/Pose) : a list of at most \
                :const:`MAX_GRASPS` grasps

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo("Action {}: Calculating grasps on object at: {}"
                      .format(self.name, str(segmented_obj.center).replace("\n", ", ")))
        self._stopped = False

        # Given the segmentation and the objects, now ask for grasps
        grasps = self._suggest_grasps_srv(cloud=segmented_obj.point_cloud).grasp_list
        self.notify_service_called(FindGraspsAction.SUGGEST_GRASPS_SERVICE_NAME)
        if len(grasps.poses) == 0:
            yield self.set_aborted(
                action=self.name,
                goal=segmented_obj,
                srv=self._suggest_grasps_srv.resolved_name,
                num_grasps=len(grasps.poses)
            )
        elif self._stopped:
            yield self.set_preempted(
                action=self.name,
                goal=segmented_obj,
                srv=self._suggest_grasps_srv.resolved_name,
                num_grasps=len(grasps.poses)
            )
        else:
            yield self.set_running()

        # If we updated status because of an error, STOP
        if self.is_preempted() or self.is_aborted():
            raise StopIteration()

        # Now that we have grasps, pairwise rank them
        grasps = self._grasps_rank_srv().grasp_list
        self.notify_service_called(FindGraspsAction.PAIRWISE_RANK_SERVICE_NAME)
        if len(grasps.poses) == 0:  # Something has gone horribly wrong
            yield self.set_aborted(
                action=self.name,
                goal=segmented_obj,
                srv=self._grasps_rank_srv.resolved_name,
                num_grasps=len(grasps.poses)
            )
        elif self._stopped:
            yield self.set_preempted(
                action=self.name,
                goal=segmented_obj,
                srv=self._grasps_rank_srv.resolved_name,
                num_grasps=len(grasps.poses)
            )
        else:
            grasps.poses = grasps.poses[:self._max_grasps]
            yield self.set_succeeded(grasps=grasps)

    def stop(self):
        self._stopped = True
