# Task Execution

This folder encapsulates the task execution interface packages that dictate the high level behaviour of the robot. The packages are:

- [`task_execution_msgs`](task_execution_msgs/) - The interface(s) between the task execution and monitoring nodes
- [`task_executor`](task_executor/) - Creates nodes with action clients, service clients, subscribers, publishers, etc. to communicate with the rest of the robot and dictate other nodes to perform the task. Also contains code to sequence the capabilities as necessary.
- [`task_monitor`](task_monitor/) - Creates nodes to monitor the progress of tasks on the task executor. In the event of task failure, the nodes in this package figure out recovery mechanisms.


## Launch

By default, the `task_executor` nodes and the `task_monitor` nodes should startup with the following command:

```bash
roslaunch task_executor fetchit.launch task_executor:=true
```

Each of the packages does have a primary launch file that is invoked by `fetchit.launch`:

- [`task_executor task_executor.launch`](task_executor/launch/task_executor.launch)
- [`task_monitor monitor.launch`](task_monitor/launch/monitor.launch)

Further details can be found in the respective packages.


## Overview

The overall architecture between the packages is shown in the following figure.

![Package Structure](doc/package_structure.png)

At a high level, the flow of control is as follows:

1. Tasks, defined as sequences of robot actions, are assigned to the `task_executor` node using an [`ExecuteActionGoal`](task_execution_msgs/action/Execute.action). The available tasks are generally defined at [`tasks.yaml`](task_executor/config/tasks.yaml)
1. The `task_executor` calls various actions, services, etc. in the robot system to accomplish the task. As part of the execution, there are 2 sources of information that the executor can use to parameterize its actions or the control flow:
    1. The `database` node contains known locations and/or joint poses in the environment that can be used as inputs for actions. The entries in the `database` are generally available in [`data.yaml`](task_executor/config/data.yaml)
    1. The `beliefs` node is designed to keep track of semantic aspects of the robot or world state through the progress of the task. Beliefs are continuous values between 0 and 1 that can be updated by any node in a non-Bayesian manner by publishing to `/execution_monitor/trace`. The beliefs that are tracked are defined at [`BeliefKeys.msg`](task_execution_msgs/msg/BeliefKeys.msg)
1. If the task succeeds, or is preempted, the `task_executor` returns the corresponding status (`actionlib_msgs/GoalStatus`) and result
1. If the `task_executor` encounters an error, it generates a [`RequestAssistanceActionGoal`](task_execution_msgs/action/RequestAssistance.action) for the `task_monitor` and then awaits a [`RequestAssistanceActionResult`](task_execution_msgs/action/RequestAssistance.action) from it, which contains a `resume_hint` on how to (or not to) proceed with the task
1. When addressing a `RequestAssistanceActionGoal`, the `task_monitor` has the option of executing recovery actions itself. It can do this through the `recovery_executor`, which is simply another instantiation of the `task_executor`
1. In order to decide the recovery strategies to use, the `task_monitor` has access to a trace of tasks and faults that have been completed through the `execution_monitor`. The latter logs [`ExecutionEvent`](task_execution_msgs/msg/ExecutionEvent.msg) messages, which are sent out on the topic `/execution_monitor/trace`
1. In addition to the `task_executor`, which publishes to `/execution_monitor/trace` the status of each of its actions as they are completed, dedicated fault monitors also publish fault statuses of different components as and when faults are detected.
