.. _targets:

*******************************************************************************
Targets
*******************************************************************************

-----------------------
Single Targets (Static)
-----------------------

Target classes are used to describe the goal condition (i.e. end condition) of a robot
for motion planning. They can be used for both Free Motion Planning (FMP) and Cartesian
Motion Planning (CMP).

The :class:`compas_fab.robots.FrameTarget` is the most common target for motion planning.
It defines the complete pose of the end-effector (or the robot flange, if no tool is attached).
A frame target is commonly created from a :class:`compas.geometry.Frame` object, or alternatively from a :class:`compas.geometry.Transformation` object.

The :class:`compas_fab.robots.PointAxisTarget` class is used for specifying a target
based on a point (`target_point`) and a Z-Axis (`target_z_axis`).
This is useful for example when the robot is using a cylindrical tool to perform a task,
for example 3D printing, welding or drilling.
In a more general case, it can be used for any tools for which the rotation
around its own Z axis is acceptable during use.
The point and the Z-Axis are defined relative to the tool coordinate frame (TCF).
The goal is (1) for the tool's TCF point to coincide with the `target_point`,
and (2) for the TCF's Z-axis to become parallel to the `target_z_axis`.
Note that the exact orientatio of the TCF is not determined until after the target is planned.

The :class:`compas_fab.robots.ConfigurationTarget` class is used to specify a target
based on a specific robot configuration (joint values).
For example, it can be used to move the robot to a taught position acquired by jogging.
Typically, the ConfigurationTarget should have the same number of joints as the planning group
of the robot. However, it is possible to specify a subset of the joints, in which
case the remaining joints are left unspecified.

The :class:`compas_fab.robots.ConstraintSetTarget` class is used to specify a list of
constraints as a planning target. This is intended for advanced users who want to create custom
combination of constraints. See :class:`compas_fab.robots.Constraint` for available
constraints. At the moment, only the ROS MoveIt planning backend supports this target type.

------------------------------------------
Waypoints Target (Multiple Point Segments)
------------------------------------------

The :class:`compas_fab.robots.Waypoint` classes are used to describe a sequence of
waypoints that the robot should pass through.