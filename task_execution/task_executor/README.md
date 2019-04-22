# Task Executor

This package executes a pick-and-place task. When there is a failure, it calls sends a request to an arbitrator module to help it decide who to contact, and then it sends a request to a help interface. When the interface signals a completion, the task execution resumes.


## Concepts

The package uses three concepts when executing a task:

- `actions`: Actions are primitive robot actions that can be executed by either calling low level controllers (such as the torso controller), or high level planners (such as move_base or MoveIt), or even higher level action servers (such as the RAIL Lab's grasp calculation pipeline). In a hierarchical task plan, actions are at the leaves of the hierarchy; in a state machine (`smach`), actions are the states.
- `ops`: Ops (operations) are code operations that do not interface with the robot's sensors or actuators, but instead operate upon the state or task plan between one action and the next.
- `tasks`: Tasks are sequences of `actions` and `ops` that can either be predefined or constructed on the fly. In a hierarchical task plan, tasks are any node that is not a leaf node; in a state machine (`smach`), tasks are state machines.

Task definitions are currently stored in [`config/tasks.yaml`](config/tasks.yaml), actions are defined in [`src/task_executor/actions`](src/task_executor/actions), and ops are defined in [`src/task_executor/ops.py`](src/task_executor/ops.py). See the section on [Syntax (TODO)](#syntax).

In order to execute a task, the package also provides a construct for the background knowledge - the database node. We currently store:

- `object_constraints` (type: [ObjectConstraints](msg/ObjectConstraints.msg)). The constraints to use when trying to recognize objects relevant to a task.
- `waypoints` (type: [Waypoint](msg/Waypoint.msg)[]). The `(x, y, theta)` poses to follow when navigating to different goals.
- `arm_gripper_pose` (type: geometry_msgs/PoseStamped). A predefined PoseStamped for the gripper.
- `arm_joint_pose` (type: [ArmJointPose](msg/ArmJointPose.msg)). A predefined set of joint angles for the arm, e.g., the tuck pose.
- `trajectories` (type: [ArmJointPose](msg/ArmJointPose.msg)[]). A predefined trajectory of arm poses to use when moving from one arm pose to another. This is often used when MoveIt! planning might create unsafe trajectories.

The knowledge that we store is often environment dependent, with the data stored for [simulation](config/simulation.yaml) different from that stored for the [robot](config/robot.yaml).

**Why not `smach`?**

If you haven't used it, [`smach`](http://wiki.ros.org/smach) is a good tool for specifying tasks in a deterministic manner. However, it is not the right tool for this project because:

1. Cancelling running states in `smach` requires the presence of a monitoring thread or a `ConcurrentState` wrapper that does the monitoring.
1. Sometimes the same actions in a very structured task need to be reused in an unstructured/poorly scheduled manner in a different context. The action and task definitions in this package try to divorce the determinism of a structured task, from the structure itself.

Despite these shortcomings, `smach` has its own benefits including a mature code API, a viewer, and widespread adoption. So I'd recommend using that as a first resort before using the definitions in this package.


## Quickstart

See the [Quickstart](../#quickstart) in the root README.


## Notes

1. Actions **cannot** be performed concurrently (by the same ROS node) at this time. However, it *should* be a trivial change to the code if we want to allow concurrent execution of actions.
1. Some actions enable or disable background behaviour. There is no current method to mutex those behaviours based on resource constraints at the moment.
