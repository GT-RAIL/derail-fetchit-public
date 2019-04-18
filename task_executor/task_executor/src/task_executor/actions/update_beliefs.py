#!/usr/bin/env python
# Updates the beliefs as specified

from __future__ import print_function, division

import rospy

from task_executor.abstract_step import AbstractStep

from assistance_msgs.msg import BeliefKeys


class UpdateBeliefsAction(AbstractStep):
    """
    Update the beliefs. Beliefs are a dictionary with the keys specified as
    either keys in BeliefKeys (recommended) or as the values themselves
    """

    def init(self, name):
        self.name = name

    def run(self, beliefs):
        rospy.loginfo("Action {}: {}".format(self.name, beliefs))

        # Lookup the belief keys as needed
        disambiguated_beliefs = {}
        for belief, value in beliefs.iteritems():
            disambiguated_beliefs[getattr(BeliefKeys, belief, belief)] = value

        # Update the beliefs
        self.update_beliefs(disambiguated_beliefs)

        # Set the result to succeeded
        yield self.set_succeeded()

    def stop(self):
        # This action cannot be stopped
        pass
