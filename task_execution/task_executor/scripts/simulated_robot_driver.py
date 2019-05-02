#!/usr/bin/env python
# Simulate the /robot_driver so that interfaces to it can operate the same on
# the real robot and in simulation

from __future__ import print_function, division

from threading import Lock

import rospy
import diagnostic_updater

from diagnostic_msgs.msg import DiagnosticStatus
from fetch_driver_msgs.msg import (RobotState, ChargerState, GripperState,
                                   JointState as FetchJointState)
from power_msgs.msg import BreakerState
from sensor_msgs.msg import JointState
from power_msgs.srv import BreakerCommand, BreakerCommandResponse
from std_srvs.srv import Trigger, TriggerResponse


# Helper function to produce diagnostic updaters
def produce_breaker_diagnostic_func(breaker):
    def diagnostic_func(stat):
        stat.summary(
            DiagnosticStatus.OK if breaker.state == BreakerState.STATE_ENABLED else DiagnosticStatus.ERROR,
            "Enabled" if breaker.state == BreakerState.STATE_ENABLED else "Disabled"
        )
        return stat
    return diagnostic_func


# This is the class that acts as the stub to the robot driver
class SimulatedRobotDriver(object):
    """
    In simulation, implement the minimum amount of logic necessary to correctly
    spoof the behaviour of the robot driver
    """

    BATTERY_FULL_VOLTAGE = 25
    BATTERY_LOW_VOLTAGE = 19.9
    BATTERY_FULL_CAPACITY = 133400
    BATTERY_LOW_CAPACITY = 6000
    BATTERY_CAPACITY_DECAY = 10  # Amount of capacity to lose per second

    GRIPPER_JOINT_NAME = 'l_gripper_finger_joint'

    def __init__(self):
        # Internal parameters for the functions of this driver
        self._publish_rate = 50  # Hz rate.

        # The state of the arm, gripper, and base breakers
        self._arm_breaker_state = BreakerState(
            name="arm_breaker",
            state=BreakerState.STATE_ENABLED
        )
        self._base_breaker_state = BreakerState(
            name="base_breaker",
            state=BreakerState.STATE_ENABLED
        )
        self._gripper_breaker_state = BreakerState(
            name="gripper_breaker",
            state=BreakerState.STATE_ENABLED
        )

        # The cached state of the robot
        self._robot_state = RobotState(
            ready=True,
            breakers=[self._arm_breaker_state, self._base_breaker_state, self._gripper_breaker_state],
            charger=ChargerState(
                state=0,          # Unknown what this actually is
                charging_mode=2,  # "Not Charging" according to the comments
                battery_voltage=SimulatedRobotDriver.BATTERY_FULL_VOLTAGE,
                battery_capacity=SimulatedRobotDriver.BATTERY_FULL_CAPACITY
            )
        )
        self._robot_state_lock = Lock()

        # The cached state of the gripper
        self._gripper_state = GripperState(ready=True)
        self._gripper_state.joints.append(FetchJointState(
            name="gripper_joint",
            control_mode=3,  # Based on values on the robot
            position=0.05,   # Default start position of open
        ))
        self._gripper_state_lock = Lock()

        # Create the diagnostic updater
        self._updater = diagnostic_updater.Updater()
        self._updater.setHardwareID("none")
        self._updater.add("arm_breaker", produce_breaker_diagnostic_func(self._arm_breaker_state))
        self._updater.add("base_breaker", produce_breaker_diagnostic_func(self._base_breaker_state))
        self._updater.add("gripper_breaker", produce_breaker_diagnostic_func(self._gripper_breaker_state))

        # Publishers
        self._robot_state_publisher = rospy.Publisher('/robot_state', RobotState, queue_size=1)
        self._gripper_state_publisher = rospy.Publisher('/gripper_state', GripperState, queue_size=1)

        # Subscribers
        self._joint_state_sub = rospy.Subscriber('/joint_states', JointState, self._on_joint_state)

        # The services to set and reset the breakers
        self._arm_breaker_service = rospy.Service("/arm_breaker", BreakerCommand, self.set_arm_breaker)
        self._base_breaker_service = rospy.Service("/base_breaker", BreakerCommand, self.set_base_breaker)
        self._gripper_breaker_service = rospy.Service("/gripper_breaker", BreakerCommand, self.set_gripper_breaker)

        # Simulation service to put the battery into low mode or not
        self._battery_low_service = rospy.Service(
            "~battery_to_low",
            Trigger,
            self._on_battery_to_level(
                SimulatedRobotDriver.BATTERY_LOW_VOLTAGE, SimulatedRobotDriver.BATTERY_LOW_CAPACITY
            )
        )
        self._battery_nominal_service = rospy.Service(
            "~battery_to_nominal",
            Trigger,
            self._on_battery_to_level(
                SimulatedRobotDriver.BATTERY_FULL_VOLTAGE, SimulatedRobotDriver.BATTERY_FULL_CAPACITY
            )
        )

    def _on_joint_state(self, msg):
        try:
            idx = msg.name.index(SimulatedRobotDriver.GRIPPER_JOINT_NAME)
            with self._gripper_state_lock:
                self._gripper_state.joints[0].position = msg.position[idx]
                self._gripper_state.joints[0].velocity = msg.velocity[idx]
                self._gripper_state.joints[0].effort = msg.effort[idx]
        except ValueError as e:
            pass

    def _on_battery_to_level(self, battery_voltage, battery_capacity):
        def service_responder(req):
            with self._robot_state_lock:
                self._robot_state.charger.battery_voltage = battery_voltage
                self._robot_state.charger.battery_capacity = battery_capacity
            return TriggerResponse(success=True)
        return service_responder

    def _calculate_robot_state(self):
        # Make sure to acquire the lock to the robot state before calling this
        # function
        self._robot_state.faulted = (
            self._arm_breaker_state.state == BreakerState.STATE_DISABLED
            or self._base_breaker_state.state == BreakerState.STATE_DISABLED
            or self._gripper_breaker_state.state == BreakerState.STATE_DISABLED
        )

    def set_arm_breaker(self, req):
        with self._robot_state_lock:
            self._arm_breaker_state.state = BreakerState.STATE_ENABLED if req.enable else BreakerState.STATE_DISABLED
            self._calculate_robot_state()
            return BreakerCommandResponse(self._arm_breaker_state)

    def set_base_breaker(self, req):
        with self._robot_state_lock:
            self._base_breaker_state.state = BreakerState.STATE_ENABLED if req.enable else BreakerState.STATE_DISABLED
            self._calculate_robot_state()
            return BreakerCommandResponse(self._base_breaker_state)

    def set_gripper_breaker(self, req):
        with self._gripper_state_lock:
            self._gripper_state.ready = req.enable
            self._gripper_state.faulted = not req.enable

        with self._robot_state_lock:
            self._gripper_breaker_state.state = BreakerState.STATE_ENABLED if req.enable else BreakerState.STATE_DISABLED
            self._calculate_robot_state()
            return BreakerCommandResponse(self._gripper_breaker_state)

    def spin(self):
        rate = rospy.Rate(self._publish_rate)
        post = rospy.Time.now()
        rate.sleep()

        while not rospy.is_shutdown():
            pre = rospy.Time.now()

            with self._robot_state_lock:
                self._robot_state.header.stamp = pre
                self._robot_state.header.seq += 1
                self._robot_state.charger.battery_capacity -= (
                    SimulatedRobotDriver.BATTERY_CAPACITY_DECAY * (pre - post).to_sec()
                )
                self._robot_state_publisher.publish(self._robot_state)

            with self._gripper_state_lock:
                self._gripper_state.header.stamp = pre
                self._gripper_state.header.seq += 1
                self._gripper_state_publisher.publish(self._gripper_state)

            self._updater.update()
            post = rospy.Time.now()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('robot_driver')
    driver = SimulatedRobotDriver()
    driver.spin()
