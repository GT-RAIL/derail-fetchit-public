# Task Executor

This package provides an action server to execute tasks. When there is a failure, it sends a request to a task monitoring module to help it recover, or not, from the problem.


## Quickstart

This section is a brief overview on launching and running tasks or actions.

### Launch

The primary means of launching the task executor is through the launch file [`fetchit.launch`](launch/fetchit.launch), which brings up the dependencies of the task executor in addition to the executor itself. The recommended means of running the executor is on 2 different terminals for ease of reading the logs:

```bash
# Start the dependencies of the executor. Remember sim:=true for simulation and
# launch_schunk:=true if we are running locally and not in simulation
roslaunch task_executor fetchit.launch start_all:=true task_executor:=false
```

```bash
# Start the task executor itself. Remember sim:=true for simulation
roslaunch task_executor fetchit.launch task_executor:=true
```

The launch file has additional args, such as `sim`, `manipulation`, `launch_schunk`, `task_config`, etc., that could be useful for launching specific dependencies or specifying config files.


### Running Tasks and Actions

Once the task executor and its dependencies are ready, we can run tasks using the [`run_task.py`](scripts/run_task.py) script:

```bash
rosrun task_executor run_task.py <task_name>
```

**Note**: Currently we do not have the capability to run tasks that require parameters. This is coming soon.

In addition to running tasks, one can also run specific actions, which is accomplished with the [`run_action.py`](scripts/run_action.py) script. This script does not need all the dependencies of the task_executor to be setup, just the dependencies of the action in question. To run:

```bash
rosrun task_executor run_action.py <action_name> <action_params>
```

The action params should be provided as a JSON string.


## Tasks: A Conceptual Overview

This package considers a robot task as a hierarchical state machine, and provides convenience capabilities for creating such a machine. To do so, it draws inspiration from drag-and-drop programming languages to create a YAML-based syntax for task (read state machine) specification that is very extensible.

To begin with, a task consists of a series of `steps`. The steps are executed sequentially, unless a loop or a branching condition is specified, in which case subsequent steps are executed based on the loop or the branch condition (but more on that later). When invoking a step, one may, depending on its type provide `params` and expect returned values, called `var`.

There are five types of steps that can be invoked:

1. `task` steps, which are other tasks, with steps that may be specified. The ability for tasks to invoke other tasks is a key feature of this package's capabilities. When instantiated as Python objects, tasks are instances of [`AbstractStep`](src/task_executor/abstract_step.py).
1. `action` steps, which are interfaces to robot capabilities. These are also instances of [`AbstractStep`](src/task_executor/abstract_step.py) and usually contain service or action clients to talk to robot capability nodes. Actions provide the basic functionality that we aim to chain together, intelligently, to specify tasks.
1. `op` steps, which are simple mathematical or programmatic operations, such as variable assignment, or variable decrement, that might be necessary to parameterize tasks and actions.
1. `choice` steps, which check the truthiness of a specified statement, and accordingly execute one step or another.
1. `loop` steps, which check the truthiness of a specified statement, and execute a specified in a loop until the statement evaluates to `false`.

The following YAML specification illustrates an example program containing all the above types of steps:

```yaml
tasks:
  # This task specification exists to show an example of invoking a simple
  # task, the loop_and_leave task
  example:
    steps:
    - task: loop_and_leave

  # Loop the greeting thrice and say goodbye
  loop_and_leave:
    steps:
    - op: assign
      params:
        var_name: num_invocations
        value: 3
      var:
      - num_invocations

    # Loop until the num_invocations variable reaches 0
    - loop: greeting_loop
      params:
        condition: var.num_invocations
        loop_body:
          task: greet
          params:
            num_invocations: var.num_invocations
          var:
          - num_invocations

    # Technically this is unnecessary; it's for illustrative purposes
    - choice: say_goodbye_choice
      params:
        condition: var.num_invocations
        if_false:
          action: speak
          params:
            text: "Goodbye human"
            affect: sad
            async: true

  # A task to greet the person. Also decrement the number of calls
  greet:
    params:
    - num_invocations

    var:
    - num_invocations

    steps:
    - action: speak
      params:
        text: "Greetings human"

    # In the current definition of `speak`, this might fail, but let's ignore
    # that detail for now. The key point is how to reference a param
    - action: speak
      params:
        text: params.num_invocations

    - op: decrement
      params:
        var_name: num_invocations
      var:
      - num_invocations
```

One could call this program with `rosrun task_executor run_task.py example`, at which point the program would have the robot speak "Greetings human" and the value of the variable `num_invocations` three times before speaking "Goodbye human" and exiting.


### task

Tasks are specified in the YAML syntax; see the example above. In the syntax, each task is assigned a unique key with each task specification having 1 required field and 2 optional fields:

* steps (*Required*) : the list of steps, with each step a dictionary definition of one of the five step types
* params (*Optional*) : a list of the names of the parameters that the task should expect as keyword arguments. We do not currently check before task invocation if the expected params match the provided ones
* var (*Optional*) : a list of the names of the variables in a task's current execution context that it's parent task can expect from it. We do check after task execution if the variables provided by a task match the expected variables from it.

When invoked, if a task has `params` or `var` defined, then those keys must be defined for the invocation. In the example tasks above, consider the difference between the invocation of `loop_and_leave` and `greet`.

### action

Actions are specified as Python modules in [`task_executor.actions`](src/task_executor/actions/) and they usually instantiate service clients, action clients, subscribers, etc. to interface with the robot's ROS nodes. In order to refer to an action by name in a task specification, the action's `class` must be imported in [`__init__.py`](src/task_executor/actions/__init__.py) and the imported `class` must be associated with an action name in the `default_actions_dict`.

Depending on the action, there are 2 optional fields that must be present when the action is invoked from a task:

* params (*Optional*) : all parameters of an action class's `run` method. Those params that do not take default arguments must be specified with the `params` field in the task. Python checks this by default.
* var (*Optional*) : a list of expected variables from the action if any. If there is a mismatch between the variables expected from an action and those that it provides, we throw an error.

In the example above, the [`speak`](src/task_executor/actions/speak.py) action has one required keyword param - `text` - and two optional keyword params - `affect` and `async`. Additionally, it does not return variables, so therefore has no `var` key in its step specifications.

### op

Ops are operations that are specified in [`task_executor.ops`](src/task_executor/ops.py). They are general helper functions for manipulating variables during the course of the task. Check out the API documentation for more details.

### choice

Branching is performed by the `choice` step. When provided, the step requires:

* a label, specified in the task steps as `choice: <label>` (see example), in order to make debugging easier.
* a `condition` param that has a value that can be truthy or falsy (the value need not strictly be a `bool`). See [condition](#condition) for more details.

Given the condition specification, the `choice` takes as params two additional optional params - `if_true` and `if_false`, which if specified, must contain either `task`, `action`, or `op` steps. The step specified in `if_true` is run if the `condition` evaluates to true, else the step in `if_false` is executed.

### loop

In order to loop, we use the `loop` step. When provided, the step requires:

* a label, specified in the task steps as `loop: <label>` (see example), in order to make debugging easier
* a `condition` param that has a value that can be truthy or falsy. See [condition](#condition) for more details.
* a `loop_body` param that contains either a `task`, `action`, or `op` step

Given the condition, the `loop` will execute the step specified in `loop_body` until `condition` evaluates to `false`.

### condition

The `condition` is a param that is required by both the `choice` and `loop` steps in the task and it decides the behaviour of those steps. The value of the condition param can be any string usable in [`eval`](https://www.programiz.com/python-programming/methods/built-in/eval). Some valid `condition` strings are:

* true
* params.boolean_param
* "( params.param1 is None or params.param2 is not None )"
* params.param1 == 1
* "str.upper(' params.object_key ').strip() in ['SMALL_GEAR', 'LARGE_GEAR']"

Note that:

* there is minimal parsing logic to the value of the condition param, so please do not put malicious code in there
* if there is a `params.*` or `var.*` variable that you wish to use in the condition string, then make sure that the variable expression is separated by a 'space'
* follow YAML [string specification paradigms](http://blogs.perl.org/users/tinita/2018/03/strings-in-yaml---to-quote-or-not-to-quote.html)
* do NOT use the YAML keyword `null`. The ROS parameter server does not like that keyword


## Additional Nodes

In addition to executing tasks, the package has nodes to provide

1. Database-like functionality for task parameters: `/database`
1. Semantic tracking of robot, task, or world state that can be used to alter task execution or perform recoveries: `/beliefs`

### Database

The database node reads in the YAML file [data.yaml](config/data.yaml), which specifies five types of information (at present):

1. `waypoints` in the map that the base can move to
1. `object_constraints`, which are deprecated for this challenge
1. `arm_gripper_poses`, which are 6dof poses that are primarily designed to specify desired end effector poses for the arm, but can also be used to specify where to look.
1. `arm_joint_poses`, which are known joint poses of the arm, for example `tuck`
1. `trajectories`, which are a series of known joint poses that can be sent to the arm. Planning and execution from one joint pose to the next is accomplished by MoveIt!

Actions that are capable of interpreting the information in the database can, given a key, query waypoints, joint poses, etc. through service calls.

### Beliefs

Beliefs are designed to track the semantic state of things. More on them later.
