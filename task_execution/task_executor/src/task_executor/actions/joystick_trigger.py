#!/usr/bin/env python
# The trigger based on the joystick buttons

import rospy

from task_executor.abstract_step import AbstractStep

from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Joy


class JoystickTriggerAction(AbstractStep):
    """
    A trigger returns a true or false depending on the logic in the trigger
    """

    JOY_TOPIC = "/joy"
    ACCEPT_BUTTON_IDX = 13
    REJECT_BUTTON_IDX = 15
    RESTART_BUTTON_IDX = 12

    def init(self, name):
        self.name = name

        # Internal variables to keep track of trigger state
        self._choice = None          # The choice made by the person
        self._make_a_choice = False  # Indicates if we need to ask the human
        self._binarize = True        # Returned value should be a boolean
        self._joy_sub = rospy.Subscriber(
            JoystickTriggerAction.JOY_TOPIC,
            Joy,
            self._on_joy
        )

        # Set a stop flag
        self._stopped = False

    def run(self, timeout=0.0, binarize=True):
        # Timeout of 0 implies infinite
        rospy.loginfo("Action {}: Waiting for a trigger on Joystick within time {}s"
                      .format(self.name, timeout))

        # Set the flags for the wait
        self._stopped = False
        self._choice = None
        self._binarize = binarize
        self._make_a_choice = True
        start_time = rospy.Time.now()

        # Then wait for the trigger
        while self._make_a_choice:
            if timeout > 0 and rospy.Time.now() - start_time > rospy.Duration(timeout):
                yield self.set_aborted(action=self.name, timeout=timeout)
                raise StopIteration()

            if self._stopped:
                yield self.set_preempted(action=self.name, timeout=timeout)
                raise StopIteration()

            yield self.set_running()

        # Yield a success or preempted based on the stopped variable
        if self._stopped:
            yield self.set_preempted(action=self.name, timeout=timeout)

        yield self.set_succeeded(choice=self._choice)

    def stop(self):
        self._stopped = True
        self._make_a_choice = False

    def _on_joy(self, joy_msg):
        # If there is no choice to be made, then don't make a choice
        if not self._make_a_choice:
            return

        # We need to make a choice
        if joy_msg.buttons[JoystickTriggerAction.ACCEPT_BUTTON_IDX] > 0:
            self._choice = True if self._binarize else JoystickTriggerAction.ACCEPT_BUTTON_IDX
        elif joy_msg.buttons[JoystickTriggerAction.REJECT_BUTTON_IDX] > 0:
            self._choice = False if self._binarize else JoystickTriggerAction.REJECT_BUTTON_IDX
        elif joy_msg.buttons[JoystickTriggerAction.RESTART_BUTTON_IDX] > 0:
            self._choice = None if self._binarize else JoystickTriggerAction.RESTART_BUTTON_IDX

        # Book-keeping
        if self._choice is not None:
            self._make_a_choice = False
            self.notify_topic_message(JoystickTriggerAction.JOY_TOPIC, joy_msg)
