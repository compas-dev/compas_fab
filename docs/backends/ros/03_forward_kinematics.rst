.. _ros_forward_kinematics:

*******************************************************************************
Forward Kinematics
*******************************************************************************

.. note::

   The description on this page is specific to the MoveIt! planner backend,
   see :ref:`Core Concepts: Forward Kinematics<forward_kinematics>`
   for general information on inverse kinematics.

.. currentmodule:: compas_fab.backends

:class:`MoveItPlanner` works with :class:`RosClient` to provides forward
kinematics capability using the MoveIt! planner backend.
There are two methods available, which caters to different use cases.
Both methods expects a :class:`~compas_fab.robots.RobotCellState` as input.
From which the :class:`~compas_robots.Configuration` of the robot is considered,
and the state of any attached tools or rigid bodies is taken into account.

The :class:`RobotCellState.robot_configuration<compas_fab.robots.RobotCellState>`
attribute must be a :ref:`full configuration<configuration>` of the robot.
The joint values must be within the joint limits, as defined in the robot model.

Available Methods
=================

.. currentmodule:: compas_fab.backends

The :meth:`MoveItPlanner.forward_kinematics` method allows for
the calculation of Cartesian poses (:class:`~compas.geometry.Frame`) of the robot's
Planner Coordinate Frame (PCF), an attached tool's Tool Coordinate Frame (TCF), or
an attached rigid body's Object Coordinate Frame (OCF). Users must specify the target
mode with the :class:`~compas_fab.robots.TargetMode` input.

The :meth:`MoveItPlanner.forward_kinematics_to_link` method allows
for the calculation of Cartesian poses of a specific link in the robot's kinematic chain.

Planner Behavior
================

No collision checking is performed in the forward kinematics calculation. Even if the
robot is in a collision state, the calculation will still be performed and the result
will be returned.

Examples
========

The following example shows the :meth:`MoveItPlanner.forward_kinematics`
using the UR5 robot loaded from the running ROS server.

.. literalinclude :: files/03_forward_kinematics.py
   :language: python

The following example shows a similar approach to the previous example, but
the :meth:`MoveItPlanner.forward_kinematics_to_link` method allows
you to specify a specific link in the robot's kinematic chain.

.. literalinclude :: files/03_forward_kinematics_to_link.py
   :language: python

The following example show forward kinematics calculation using a robot cell loaded
from the RobotCellLibrary. The robot cell contains a UR5 robot, the same as the robot
running in the ROS MoveIt! planner. The robot cell state contains an attached tool and a
workpiece, showing the effect of the three :class:`~compas_fab.robots.TargetMode` options.

.. literalinclude :: files/03_forward_kinematics_target_mode.py
   :language: python

