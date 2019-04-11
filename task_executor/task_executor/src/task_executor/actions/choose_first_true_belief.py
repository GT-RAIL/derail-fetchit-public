#!/usr/bin/env python
# Make a choice based on beliefs

from __future__ import print_function, division

import rospy

from task_executor.abstract_step import AbstractStep

from task_execution_msgs.msg import BeliefKeys
from task_execution_msgs.srv import GetBeliefs


class ChooseFirstTrueBeliefAction(AbstractStep):
    """
    Choose the first belief among a list of beliefs, that might be true. Return
    the associated keys when a choice is made
    """

    BELIEFS_SERVICE_NAME = "/beliefs/get_beliefs"
    BELIEF_TRUE_DEFAULT_THRESHOLD_VALUE = 0.9

    def init(self, name):
        self.name = name
        self._get_beliefs_srv = rospy.ServiceProxy(
            ChooseFirstTrueBeliefAction.BELIEFS_SERVICE_NAME,
            GetBeliefs
        )

        # Set a stop flag
        self._stopped = False

        # Wait for a connection to the beliefs
        rospy.loginfo("Connecting to the beliefs service...")
        self._get_beliefs_srv.wait_for_service()
        rospy.loginfo("...beliefs service connected")

    def run(self, belief_keys):
        # belief_keys is a list of dictionaries consisting of the belief, and
        # corresponding return values when a belief is chosen
        rospy.loginfo("Action {}: Checking beliefs {}".format(self.name, [k['key'] for k in belief_keys]))
        self._stopped = False
        yield self.set_running()  # Check the status of the server

        # Fetch the beliefs and create a dictionary
        beliefs = self._get_beliefs_srv([k['key'] for k in belief_keys])
        self.notify_service_called(ChooseFirstTrueBeliefAction.BELIEFS_SERVICE_NAME)
        beliefs = { b: v for (b, v) in zip([k['key'] for k in belief_keys], beliefs.values) }
        yield self.set_running()  # Check the status of the server
        if self._stopped:
            yield self.set_preempted(
                action=self.name,
                goal=belief_keys,
                srv=ChooseFirstTrueBeliefAction.BELIEFS_SERVICE_NAME,
                beliefs=beliefs
            )
            raise StopIteration()

        # Choose the first belief index that's true
        belief_found = False
        for belief_key in belief_keys:
            if self._stopped:
                yield self.set_preempted(
                    action=self.name,
                    goal=belief_keys,
                    srv=ChooseFirstTrueBeliefAction.BELIEFS_SERVICE_NAME,
                    beliefs=beliefs
                )
                raise StopIteration()

            if beliefs[belief_key['key']] >= belief_key.get(
                'threshold',
                ChooseFirstTrueBeliefAction.BELIEF_TRUE_DEFAULT_THRESHOLD_VALUE
            ):
                belief_found = True
                break

            yield self.set_running()

        # Yield a success if we've found a belief, else yield a failure
        if belief_found:
            rospy.loginfo("Action {}: Chosen belief - {}".format(self.name, belief_key['key']))
            yield self.set_succeeded(**belief_key['values'])
        else:
            yield self.set_aborted(
                action=self.name,
                goal=belief_keys,
                result=beliefs
            )

    def stop(self):
        self._stopped = True
