.. _backends:

********************************************************************************
Working with Planning Backends
********************************************************************************

.. highlight:: bash

.. currentmodule:: compas_fab

**COMPAS FAB** provides access to multiple robotic platforms as backends for
planning motions. As a middle layer between the user and the backend, **COMPAS FAB**
provides a unified interface to interact with the different backends.

The workflow of using the planning backends are similar across the different backends:

#. Create :class:`backends.RosClient` and :class:`backends.MoveItPlanner` instances.

#. Create a :class:`robots.RobotCell` instance to represent the robot and its environment.

   #. Include :class:`compas_robots.RobotModel` instance

   #. Include :class:`compas_robots.ToolModel` instance(s) and :class:`robots.RigidBody` instance(s).

#. Construct the planning problem

   #. Create :class:`robots.RobotCellState` to model the current state of the robot cell.

   #. Create a :class:`robots.Target` or :class:`robots.Waypoints` instance to represent the target pose.

#. Call the corresponding planning method and process the result from the planning method.

   #. Serialize the configuration (IK result) or trajectory (Motion Planning result)

   #. Convert the result to a format that can be used by the robot controller.

   #. Visualize the result in a 3D viewer or CAD software.


The following sections provide detailed information on how to work with the different backends.

.. toctree::
    :maxdepth: 2
    :titlesonly:
    :glob:

    backends/ros
    backends/pybullet
    backends/analytical_kinematics
    backends/analytical_pybullet


