#!/usr/bin/env python
# The operations that can be specified in the YAML

from __future__ import print_function, division


# Private helper functions

def _get_heap_for_var_name(var_name, variables, params):
    """Given a variable name, return the 'heap' that contains it"""
    if var_name in variables:
        return variables
    elif var_name in params:
        return params
    else:
        return None


# The Operations. All operations receive a current_params and current_variables

def assign(var_name, value, current_params, current_variables):
    """
    Assigns the indicated `value` to a variable with name `var_name`

    Args:
        var_name (str) : Name of the variable to assign the value to
        value (any) : Value to assign to that variable
    Returns:
        A dictionary with { var_name: value }
    """
    return { var_name: value }

def decrement(var_name, current_params, current_variables):
    """
    Decrements the value of variable `var_name` in the `current_variables`. The
    value must exist and must be an integer.

    Args:
        var_name (str) : Name of the variable to decrement
    Returns:
        A dictionary with the var_name value in current_variables decremented
    """
    heap = _get_heap_for_var_name(var_name, current_variables, current_params)
    return { var_name: heap[var_name] - 1 }

def make_boolean(var_name, bool_name, current_params, current_variables):
    """
    Make the value of a variable in the current variables a boolean.

    Args:
        var_name (str) : Name of the variable to binarize
        bool_name (str) : Name of the binarized version of the variable
    Returns:
        A dictionary with the { bool_name: bool(var_name) }
    """
    heap = _get_heap_for_var_name(var_name, current_variables, current_params)
    return { bool_name: bool(heap[var_name]) }

def negate(var_name, negate_name, current_params, current_variables):
    """
    Negate the current value of var_name => not var_name

    Args:
        var_name (str) : Name of the variable to negate
        negate_name (str) : Name of the variable to contain the negation
    REturns:
        A dictionary with the { var_name: not var_name }
    """
    heap = _get_heap_for_var_name(var_name, current_variables, current_params)
    return { negate_name: not heap[var_name] }
