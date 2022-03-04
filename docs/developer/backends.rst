.. _architecture:

*******************************************************************************
Backend clients
*******************************************************************************

This document details the architecture used to implement backend clients and
backend features.

To maintain consistency from one backend client to another and to promote
modularity, we make use of several interfaces. Any new backend client
should inherit from :class:`~compas_fab.backends.interfaces.ClientInterface` and
make use of the :class:`~compas_fab.backends.interfaces.PlannerInterface`.
Methods for connecting, disconnecting, and generally managing the client state
are a part of the client, while any methods for planning, scene management or
kinematics are attached to the planner.  Eventually, methods for
execution and control will be included in the ``ControlInterface``,
but for now, such methods and attributes will be left with the
client.

The ``PlannerInterface`` serves as a template for any client-specific
planner, providing default behavior for each of the
methods listed within.  When a developer wishes to override any
of these defaults, they should make use of the appropriate backend
feature interface from ``backends/interfaces.py``.  The file
``interfaces.py`` consists of a collection of classes, any
implementation of which is callable through its ``__call__`` magic
method.  For example:

.. code-block:: python

    from compas.geometry import Frame
    from compas_fab.backends.interfaces import InverseKinematics

    class ExampleInverseKinematics(InverseKinematics):
        def inverse_kinematics(self, robot,
                               frame_WCF,
                               start_configuration=None,
                               group=None,
                               options=None):
            # insert fancy code here
            pass

can be instantiated and called in the following manner:

.. code-block:: python

    calculate_example_ik = ExampleInverseKinematics()
    frame = Frame([0, 0, 0], [1, 0, 0], [0, 1, 0])
    ik_result = calculate_example_ik(robot, frame)
    # or equivalently:
    ik_result = calculate_example_ik.inverse_kinematics(robot, frame)


These backend feature interfaces exist in part to enforce a common
signature across all implementations of, say,
``inverse_kinematics`` for greater end-user ease.  Please adhere to the
types listed for the arguments and return values listed in the documentation
for the backend features as much as possible.

These interfaces as exist to allow mixing and matching of the backend
features of various clients to suit the performance and overhead
requirements of the end-user.  To illustrate this last point, consider the
following example, where the backend of ``ClientA`` is very efficient at
computing inverse kinematics and has no feature to plan motion, while the
backend of ``ClientB`` is slow to compute inverse kinematics but can plan motion:

.. code-block:: python

    with ClientA() as client_a, ClientB() as client_b:
        inverse_kinematics = ClientAInverseKinematics(client_a)
        plan_motion = ClientBPlanMotion(client_b)

Here we can assign the inverse kinematics to be calculated by the backend
of ``ClientA``, while the motion planning is calculated by the backend of
``ClientB``.  (We assume ``ClientA`` and ``ClientB`` inherit from
``ClientInterface`` and that ``ClientAInverseKinematics`` and
``ClientBPlanMotion`` inherit from ``InverseKinematics`` and
``PlanMotion``, resp.)

Backend interfaces
==================

.. automodule:: compas_fab.backends.interfaces
