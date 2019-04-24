# Task Executor

This package provides an action server to execute tasks. When there is a failure, it sends a request to a task monitoring module to help it recover, or not, from the problem.


## Conceptual Overview

This package considers a robot task as a hierarchical state machine, and provides convenience capabilities for creating such a machine. To do so, it draws inspiration from drag-and-drop programming languages to create a YAML-based syntax for task (read state machine) specification that is very extensible.

To begin with, a task consists of a series of `steps`. The steps are executed sequentially, unless a loop or a branching condition is specified, in which case subsequent steps are executed based on the loop or the branch condition (but more on that later). When invoking a step, one may, depending on its type provide `params` and expect returned values, called `var`.

There are five types of steps that can be invoked:

1. `task` steps, which are other tasks, with steps that may be specified. The ability for tasks to invoke other tasks is a key feature of this package's capabilities. When instantiated as Python objects, tasks are instances of [`AbstractStep`](src/task_executor/abstract_step.py).
1. `action` steps, which are interfaces to robot capabilities. These are also instances of [`AbstractStep`](src/task_executor/abstract_step.py) and usually contain service or action clients to talk to robot capability nodes. Actions provide the basic functionality that we aim to chain together, intelligently, to specify tasks.
1. `op` steps, which are simple mathematical or programmatic operations, such as variable assignment, or variable decrement, that might be necessary to parameterize tasks and actions.
1. `condition` steps, which check the truthiness of a specified variable, and accordingly execute one step or another.
1. `loop` steps, which check the truthiness of a specified variable, and execute a specified in a loop until the variable evaluates to `false`.

### tasks

Tasks are specified in the YAML syntax, see [`tasks.yaml`](config/tasks.yaml). In the syntax, each task is assigned a unique key with each task specification having 1 required field and 2 optional fields:

* steps (*Required*) : the list of steps, with each step a dictionary definition of one of the five step types
* params (*Optional*) : a list of the names of the parameters that the task should expect as keyword arguments. We do not currently check before task invocation if the expected params match the provided ones
* var (*Optional*) : a list of the names of the variables in a task's current execution context that it's parent task can expect from it. We do check after task execution if the variables provided by a task match the expected variables from it.

When invoked, if a task has `params` or `var` defined, then those keys must be defined for the invocation.

### actions

Actions are specified as Python modules in [`task_executor.actions`](src/task_executor/actions/) and they usually instantiate service clients, action clients, subscribers, etc. to interface with the robot's ROS nodes. In order to refer to an action by name in a task specification, the action's `class` must be imported in [`__init__.py`](src/task_executor/actions/__init__.py) and the imported `class` must be associated with an action name in the `default_actions_dict`.

Depending on the action, there are 2 optional fields that must be present when the action is invoked from a task:

* params (*Optional*) : all parameters of an action class's `run` method. Those params that do not take default arguments must be specified with the `params` field in the task. Python checks this by default.
* var (*Optional*) : a list of expected variables from the action if any. If there is a mismatch between the variables expected from an action and those that it provides, we throw an error.


## OLD STUFF

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

Despite these shortcomings, `smach` has its own benefits including a mature code API, a viewer, and widespread adoption.
