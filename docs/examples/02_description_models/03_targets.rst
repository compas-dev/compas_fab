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

The :class:`compas_fab.robot.FrameTarget` is the most common target for motion planning.
It defined the complete pose of the end-effector (or the robot flange, if no tool is attached).
The FrameTarget is commonly created from a Frame object, or alternatively from a a Transformation object.

The :class:`compas_fab.robot.PointAxisTarget` class is used for specifying a target
based on a point and a Z-Vector. This is useful for example when the robot is using a
cylinrical tool to perform a task, for example 3D printing,welding or drilling.
The point and the vector is defined relative to the Z-axis of the tool coordinate
frame (TCF). The goal is for the tool's TCF point to coincide with the `target_point`,
and the TCF's Z-axis is parallel to the Z vector. Note that the exact orientation
of the TCF is not determined until after the target is planned.

The :class:`compas_fab.robot.ConfigurationTarget` class is used to specify a target
based on a robot configuration. This is useful for example when the robot is
required to move to a specific joint configuration. This is useful for example
when the position of the robot is aquired by teaching (i.e. jogging). Typically,
the ConfigurationTarget should have the same number of joints as the planning group
of the robot. However, it is possible to specify a subset of the joints, in which
case the remaining joints are left unspecified. Note that ConfigurationTarget does
not take into account whether a tool is attached or not.
The pose of the end-effector and the robot flange is determined by forward kinematics.

The :class:`compas_fab.robot.ConstraintSetTarget` class is used to specify a list of
ROS MoveIt constraints. This is intended for advanced users who want to create custom
combination of constraints. See :class:`compas_fab.robots.Constraint` for available
constraints.

----------------------------
Multiple Tragets (Waypoints)
----------------------------




