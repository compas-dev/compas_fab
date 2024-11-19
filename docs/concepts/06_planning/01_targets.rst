
*******************************************************************************
Targets and Waypoints
*******************************************************************************

.. _targets:

-----------------------
Targets (Single Goal)
-----------------------

Target classes are used to describe the goal condition (i.e. end condition) of a
robot for motion planning. They can be used for Motion Planning with
:class:`compas_fab.backends.interfaces.PlannerInterface` that supports
:meth:`compas_fab.backends.interfaces.BackendFeature.plan_motion`.

.. _target_mode:

-----------
Target Mode
-----------

The TargetMode class is used to specify how the target is interpreted by the planner.
There are three target modes available:

- :attr:`compas_fab.robots.TargetMode.ROBOT`
- :attr:`compas_fab.robots.TargetMode.TOOL`
- :attr:`compas_fab.robots.TargetMode.WORKPIECE`

.. _frame_target:

------------
Frame Target
------------

The :class:`compas_fab.robots.FrameTarget` is the most common target for motion
planning. It defines the complete pose of the referenced object (as defined by
TargetMode). A frame target is created from a :class:`compas.geometry.Frame` object,
or alternatively from a :class:`compas.geometry.Transformation` object.

Position and orientation accuracy can be specified using the `tolerance_position` and
`tolerance_orientation` attributes. These values are used by the planner to determine
if the goal has been reached. The tolerance values are in meters and radians respectively
and are not affected by the native scale.

.. _point_axis_target:

-----------------
Point-Axis Target
-----------------

The :class:`compas_fab.robots.PointAxisTarget` class is used for specifying a target
based on a point (`target_point`) and a Z-Axis (`target_z_axis`).
This is useful for example when the robot is using a cylindrical tool to perform a task,
for example 3D printing, welding or drilling.
In a more general case, it can be used for any tools for which the rotation
around its own Z axis is acceptable during use.
The point and the Z-Axis are defined relative to the tool coordinate frame (TCF).
The goal is (1) for the tool's TCF point to coincide with the `target_point`,
and (2) for the TCF's Z-axis to become parallel to the `target_z_axis`.
Note that the exact orientation of the TCF is not determined until after the target is planned.

.. _configuration_target:

--------------------
Configuration Target
--------------------

The :class:`compas_fab.robots.ConfigurationTarget` class is used to specify a target
based on a specific robot configuration (joint values).
For example, it can be used to move the robot to a taught position acquired by jogging.
Typically, the ConfigurationTarget should have the same number of joints as the planning group
of the robot. However, it is possible to specify a subset of the joints, in which
case the remaining joints are left unspecified.

.. _constraint_set_target:

---------------------
Constraint Set Target
---------------------

The :class:`compas_fab.robots.ConstraintSetTarget` class is used to specify a list of
constraints as a planning target. This is intended for advanced users who want to create custom
combination of constraints. See :class:`compas_fab.robots.Constraint` for available
constraints. At the moment, only the ROS MoveIt planning backend supports this target type.




.. _waypoints:

------------------------------------------
Waypoints (Multiple Points / Segments)
------------------------------------------

Waypoints classes are used to describe a sequence of
waypoints that the robot should pass through in a planned motion. They are similar to Targets classes
but contain a list of targets instead of a single target, which is useful for tasks such as
drawing, welding or 3D printing.
They can be used for Cartesian Motion Planning with :meth:`compas_fab.robots.Robot.plan_cartesian_motion`.

The :class:`compas_fab.robots.FrameWaypoints` is the most common waypoint for Cartesian motion planning.
It defines a list of complete pose for the end-effector (or the robot flange, if no tool is attached).
It is created by a list of :class:`compas.geometry.Frame` objects or alternatively from a list of
:class:`compas.geometry.Transformation` objects.

The :class:`compas_fab.robots.PointAxisWaypoints` class is used for specifying a list of waypoints based on
the Point-Axis concept used in the :class:`compas_fab.robots.PointAxisTarget`. Compared to
:class:`~compas_fab.robots.FrameWaypoints`, this class allows for specifying targets where the rotation
around the Z-axis is not fixed. This is useful for example when the robot is using a cylindrical tool
to perform a task, for example 3D printing, welding or drilling. The freely rotating axis is defined relative
to the Z-axis of the tool coordinate frame (TCF). Note that the orientation of the tool
at the end of the motion is not determined until after the motion is planned.

