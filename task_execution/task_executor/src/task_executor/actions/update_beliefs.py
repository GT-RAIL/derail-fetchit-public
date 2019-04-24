#!/usr/bin/env python
# Updates the beliefs as specified

from __future__ import print_function, division

import rospy

from task_executor.abstract_step import AbstractStep

from task_execution_msgs.msg import BeliefKeys


class UpdateBeliefsAction(AbstractStep):
    """
    Update the desired beliefs to desired values.
    """

    def init(self, name):
        self.name = name

    def run(self, beliefs):
        """
        The run function for this step

        Args:
            beliefs (dict) : a dictionary of known \
                ``task_execution_msgs/BeliefKeys`` and the associated values, \
                which are floats between 0 and 1, that they should be set to

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
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
