ops
===

.. note::

    When specifying the parameters for ``op`` in the :doc:`task_executor.tasks`,
    we do not need to include the parameters ``current_params`` and
    ``current_vars`` because those parameters are automatically injected into
    the ``op`` function by the :py:class:`task_executor.tasks.Task`. However,
    that also means that when defining a new ``op`` function in
    :py:mod:`task_executor.ops`, we need to make sure that the function includes
    those parameters in the definition.


.. automodule:: task_executor.ops
    :members:
    :undoc-members:
    :show-inheritance:
