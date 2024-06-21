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
planner and contains all unified planning function signatures.
Developers wishing to develop new planner features should inherit from the
appropriate backend feature interface from ``backends/interfaces/backend_features.py``.
These features are then composed into a single class that inherits from
``PlannerInterface``.
For example:

.. code-block:: python

    from compas.geometry import Frame
    from compas_fab.backends.interfaces import InverseKinematics
    from compas_fab.backends.interfaces import PlannerInterface

    class ExampleInverseKinematics(InverseKinematics):
        def inverse_kinematics(self, robot,
                               frame_WCF,
                               start_configuration=None,
                               group=None,
                               options=None):
            # The backend features have access to the instance of the client
            print(type(self.client))
            # insert fancy code here
            pass


    # Now we can create a custom planner using our newly implemented backend feature
    class ExamplePlanner(ExampleInverseKinematics, PlannerInterface):
        pass

The planner can be instantiated and called in the following manner:

.. code-block:: python

    # Instantiate the client and change its planner to the ExamplePlanner
    client = MyExamplePlanner(robot)
    client.planner = ExamplePlanner(client)
    # Call the inverse kinematics method as usual
    frame = Frame([0, 0, 0], [1, 0, 0], [0, 1, 0])
    ik_result = robot.inverse_kinematics(robot, frame)


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

    with ClientA(robot) as client_a, ClientB(robot) as client_b:
        inverse_kinematics = client_a.planner.inverse_kinematics
        plan_motion = client_b.planner.plan_motion

        frame = Frame([0, 0, 0], [1, 0, 0], [0, 1, 0])
        ik_result = inverse_kinematics(frame)

        start_configuration = robot.zero_configuration()
        goal_configuration = robot.random_configuration()
        motion_plan = plan_motion(start_configuration, goal_configuration)

Here we can assign the inverse kinematics to be calculated by the backend
of ``ClientA``, while the motion planning is calculated by the backend of
``ClientB``.  (We assume ``ClientA`` and ``ClientB`` inherit from
``ClientInterface`` and that ``ClientAInverseKinematics`` and
``ClientBPlanMotion`` inherit from ``InverseKinematics`` and
``PlanMotion``, resp.)

Backend interfaces
==================

.. automodule:: compas_fab.backends.interfaces

Implemented backend features
============================

The following backend features are implemented for the ROS backend:

.. automodule:: compas_fab.backends.ros.backend_features

The following backend features are implemented for the PyBullet backend:

.. automodule:: compas_fab.backends.pybullet.backend_features

