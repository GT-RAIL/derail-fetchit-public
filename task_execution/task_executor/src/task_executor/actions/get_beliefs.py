#!/usr/bin/env python
# Get the values of the specified beliefs

from __future__ import print_function, division

import rospy

from task_executor.abstract_step import AbstractStep

from task_execution_msgs.msg import BeliefKeys
from task_execution_msgs.srv import GetBeliefs, GetBeliefsRequest


class GetBeliefsAction(AbstractStep):
    """
    Gets specified beliefs from :class:`task_executor.beliefs.BeliefsServer`.
    """

    BELIEFS_SERVICE_NAME = '/beliefs/get_beliefs'

    def init(self, name):
        self.name = name
        self._get_beliefs_srv = rospy.ServiceProxy(GetBeliefsAction.BELIEFS_SERVICE_NAME, GetBeliefs)

        # Set a stop flag
        self._stopped = False

        # Wait for a connection to the beliefs
        rospy.loginfo("Connecting to the beliefs service...")
        self._get_beliefs_srv.wait_for_service()
        rospy.loginfo("...beliefs service connected")

    def run(self, belief_keys):
        """
        The run function for this step

        Args:
            belief_keys (list of str) : A list of belief keys present in \
                ``task_execution_msgs/BeliefKeys`` to fetch

        Yields:
            belief_values (float) : The value for each of the specified \
                belief_keys that was passed into the action

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        rospy.loginfo("Action {}: Fetching beliefs {}".format(self.name, belief_keys))

        # Sanity check that the requested keys
        for key in belief_keys:
            assert isinstance(key, str) and (key.upper() in dir(BeliefKeys)), \
                "Unknown BeliefKey: {}".format(key)

        # Fetch the beliefs and create the return dictionary
        req = GetBeliefsRequest(beliefs=tuple(belief_keys))
        beliefs = self._get_beliefs_srv(req)
        self.notify_service_called(GetBeliefsAction.BELIEFS_SERVICE_NAME)
        beliefs = { b: v for (b, v) in zip(beliefs.beliefs, beliefs.values) }
        yield self.set_running()  # Check the status of the server

        # Return the appropriate dictionary
        if self._stopped:
            yield self.set_preempted(
                action=self.name,
                goal=belief_keys,
                srv=GetBeliefsAction.BELIEFS_SERVICE_NAME,
                beliefs=beliefs
            )
        else:
            yield self.set_suceeded(**beliefs)

    def stop(self):
        self._stopped = True
