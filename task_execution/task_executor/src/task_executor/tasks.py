#!/usr/bin/env python
# The task executor executes a task that is defined in a YAML config file

from __future__ import print_function, division

import rospy

from task_executor.abstract_step import AbstractStep
from task_executor import ops


# A class detailing how a `Task` should execute

class TaskContext(object):
    """
    A structure that's used to keep track of task state in the event of a
    failure. Based on a ``task_execution_msgs/RequestAssistanceResult``
    resume hint, the :class:`Task` can use the information in this structure to
    decide how to continue execution.
    """

    def __init__(self, start_idx=0, restart_child=True):
        """
        Args:
            start_idx (int) : Must be >= 0. The step in the task spec from where
                execution should start
            restart_child (bool) : If the current start_idx is a task, should
                that be restarted, or should it be resumed from its current
                location?
        """
        assert start_idx >= 0
        self.start_idx = start_idx
        self.restart_child = restart_child

    def __str__(self):
        return "(Start: {}, Restart: {})".format(self.start_idx, self.restart_child)


# The actual executor of tasks

class Task(AbstractStep):
    """
    All tasks defined in :doc:`task_executor.tasks` are instantiated as
    instances of this class. The behaviour of each task in that YAML file is
    defined according to the specification in :file:`task_executor/README.md`.

    Broadly, all tasks in the YAML must include ``steps``, and may optionally
    include ``params`` and ``var`` keys. The contents of those keys define the
    behaviour of this class's :meth:`run`.
    """

    def init(self, name, tasks, actions, steps, params=[], var=[], **kwargs):
        """
        The initialization of the task's actions, and thereby its connections to
        the ROS system

        Args:
            name (str) : Name of the task
            tasks (dict) : A dictionary associating a task name to an instance
                of this class. This allows tasks to call each other ad infinitum
            actions (Actions) : The actions that are available to this task
            steps (list) : The steps in this task. Part of YAML specification
            params (list) : The expected kwargs for this task. Part of YAML
                specification
            var (list) : The expected keywords in this task's return values.
                Part of YAML specification
            kwargs (kwargs) : Additional args that could be relevant. Missing by
                default
        """

        # Knowledge of all the tasks and actions
        self.name = name
        self.tasks = tasks
        self.actions = actions

        # Specific to this task
        self.params = params
        self.var = var
        self.var_values = dict()
        self.steps = steps

        # Flag to indicate if the task is stopped
        self._stopped = False

        # State flags
        self.step_idx = -1              # The current step_idx that is running
        self.current_step_def = None    # The current step definition. If a control structure, entire def is preserved
        self.current_executor = None    # The current action/task executor

    def run(self, context, **params):
        """
        Run the task by executing its steps in order. This is a generator.

        Args:
            context (TaskContext) : An object dictating how the steps should be
                executed, especially in the event of resuming from a failure
            params (kwargs) : Keyword arguments to the task. These must match
                the keywords specified (if any) in the YAML specification.

        Yields:
            variables (dict) :
                A dictionary of variables in the task's heap as the task
                executes. The keys in the dictionary on the last ``yield``
                match those specified in the YAML specification.

        .. seealso::

            :meth:`task_executor.abstract_step.AbstractStep.run`
        """
        self.step_idx = context.start_idx
        rospy.loginfo("Task {}: EXECUTING from step {}.".format(self.name, self.step_idx))

        # First validate the params that we might have received
        if not self._validate_params(self.params, params):
            rospy.logerr(
                "Task {}: FAIL. Unexpected Params. Expected: {}. Received: {}."
                .format(self.name, self.params, params)
            )
            raise KeyError(self.name, "Unexpected Params", self.params, params)

        # Setup to run the task
        var = dict() if context.restart_child else self.var_values
        self._stopped = False

        # Go through each step of the specified task plan as appropriate
        while self.step_idx < len(self.steps):
            # Save the definition of the current step; create a local var alias
            self.current_step_def = step = self.steps[self.step_idx]
            step_name = step.get('task') or step.get('action') or \
                step.get('op') or step.get('loop') or step.get('choice')

            # First resolve any and all params for this step
            step_params = {
                name: self._resolve_param(value, var, params)
                for name, value in step.get('params', {}).iteritems()
            }
            variables = {}

            # First check to see if this a loop or choice. If so, update
            # defs accordingly. current_step_def remains unchanged here
            if step.has_key('loop'):
                condition = step_params['condition']
                rospy.loginfo("Loop {}: condition - {}".format(step_name, condition))

                # We only loop while true. If done, move to next step
                if not condition:
                    self.step_idx += 1
                    continue

                # Update the step definition and step_params
                step = step_params['loop_body']
                step_params = {
                    name: self._resolve_param(value, var, params)
                    for name, value in step.get('params', {}).iteritems()
                }
            elif step.has_key('choice'):
                condition = step_params['condition']
                rospy.loginfo("Choice {}: condition - {}".format(step_name, condition))

                # Based on the condition, update the step definition
                # to the next step
                if condition:
                    step = step_params.get('if_true')
                else:
                    step = step_params.get('if_false')

                #  If the body is not defined, then we move on to next
                if step is None:
                    rospy.loginfo("Choice {}: No task defined for condition - {}".format(step_name, condition))
                    self.step_idx += 1
                    continue

                # Update the parameters associated with the step
                step_params = {
                    name: self._resolve_param(value, var, params)
                    for name, value in step.get('params', {}).iteritems()
                }

            # Check to see if this is an op. If so, run the op
            if step.has_key('op'):
                self.current_executor = None
                variables = getattr(ops, step['op'])(
                    current_variables=var,
                    current_params=params,
                    **step_params
                )

            # Otherwise, execute the action/task:
            else:
                # Then, set the appropriate executor
                if step.has_key('action'):
                    self.current_executor = self.actions[step['action']]
                else:  # step.has_key('task')
                    # Create the child task context based on saved information
                    child_context = None
                    if self.current_executor is not None \
                            and isinstance(self.current_executor, Task) \
                            and not context.restart_child \
                            and context.start_idx == self.step_idx:
                        # Restart the child if it ends in an operation
                        child_context = TaskContext(
                            start_idx=self.current_executor.step_idx,
                            restart_child=(False or self.current_executor.current_executor is None)
                        )
                    else:
                        # restart_child or current_executor is None or
                        # current_executor is not of type Task or
                        # current_executor is the previous task in the program
                        child_context = TaskContext()

                    # Set the current_executor to the task at hand
                    self.current_executor = self.tasks[step['task']]
                    step_params['context'] = child_context

                executor = self.current_executor

                # Run and stop/yield as necessary
                for variables in executor.run(**step_params):
                    # First check to see if we've been preempted. If we have,
                    # set the preempt flag and wait for the action to return
                    if self._stopped:
                        executor.stop()
                        continue

                    # Check to see if the action/task has been preempted. If so,
                    # exit out of this loop
                    if executor.is_preempted() or executor.is_aborted():
                        break

                    # Otherwise, yield a running task
                    yield self.set_running(**variables)

                # If the reason we stopped is a failure, then return
                if executor.is_preempted():
                    rospy.logwarn(
                        "Task {}, Step {}({}): PREEMPTED. Context Keys: {}"
                        .format(self.name, self.step_idx, step_name, variables.keys())
                    )
                    yield self.set_preempted(
                        task=self.name,
                        step_idx=self.step_idx,
                        step_name=step_name,
                        context=variables
                    )
                    raise StopIteration()

                if executor.is_aborted():
                    rospy.logerr(
                        "Task {}, Step {}({}): FAIL. Context Keys: {}"
                        .format(self.name, self.step_idx, step_name, variables.keys())
                    )
                    yield self.set_aborted(
                        task=self.name,
                        step_idx=self.step_idx,
                        step_name=step_name,
                        context=variables
                    )
                    raise StopIteration()

            # Validate the variables
            if not self._validate_variables(step.get('var', []), variables):
                rospy.logerr(
                    "Task {}, Step {}({}): FAIL. Invalid Variables. Expected: {}. Received: {}."
                    .format(self.name, self.step_idx, step_name, sorted(step.get('var', [])), sorted(variables.keys()))
                )
                raise KeyError(
                    self.name, self.step_idx, step_name,
                    "Invalid Variables",
                    sorted(step.get('var', [])), sorted(variables.keys())
                )

            # Update the variables that we're keeping track of
            for name, value in variables.iteritems():
                var[name] = value

            self.var_values = var

            # Only move on if we're not a loop. Loop termination happens above
            if self.current_step_def.get('loop') is None:
                self.step_idx += 1

        # Finally, yield succeeded with the variables that should be local stack
        # of variables that we're keeping track of
        rospy.loginfo("Task {}: SUCCESS.".format(self.name))
        self.step_idx = -1
        self.current_step_def = self.current_executor = None
        yield self.set_succeeded(**{var_name: self.var_values[var_name] for var_name in self.var})

    def stop(self):
        """Preempt the task"""
        self._stopped = True

    def get_executor(self):
        """
        Return a :class:`task_executor.abstract_step.AbstractStep` that is
        either a task in the middle of an op, or an action. This method is used
        to provide context to a ``task_execution_msgs/RequestAssistanceGoal``
        """
        if self.current_executor is None:
            return self
        elif isinstance(self.current_executor, Task):
            return self.current_executor.get_executor()
        else:
            return self.current_executor

    def _validate_params(self, expected_params, actual_params):
        return sorted(actual_params.keys()) == sorted(expected_params)

    def _validate_variables(self, expected_var, actual_var):
        return sorted(actual_var.keys()) == sorted(expected_var)

    def _resolve_param(self, param, var, task_params):
        if isinstance(param, str):
            splits = param.split('.', 1)  # Split up the param

            # Check if this requires a var resolution
            if len(splits) > 1 and splits[0] == 'var':
                return var[splits[1]]

            # Check if this requires a task_param resolution
            if len(splits) > 1 and splits[0] == 'params':
                return task_params[splits[1]]

        # Otherwise, this param should be used as is
        return param
