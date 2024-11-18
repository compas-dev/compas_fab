.. _planning_group:

*******************************************************************************
Planning Group
*******************************************************************************

Planning group is a concept used to denote a group of joints (and its
corresponding links) that are used for inverse kinematic calculation and motion
planning.

Definition
==========

.. currentmodule:: compas_fab.robots

Planning groups are defined in the :class:`RobotSemantics.groups<RobotSemantics>`
and are used as inputs to motion planners to denote which joints can be moved
to reach a target.

Multiple planning groups can be defined for a RobotModel. Each planning group
is denoted with a unique name and contains a subset of the robot's links and joints.
See examples below for the dictionary structure of the `.groups` attribute.

The first link in the group is referred to as the base link, such as
in the function :meth:`RobotCell.get_base_link(group)<RobotCell.get_base_link>`.
The last link is referred to as the end_effector link, such as in the function
:meth:`RobotCell.get_end_effector_link(group)<RobotCell.get_end_effector_link>`.


Application
===========

Many industrial robots are serial manipulators and can be represented by a single
planning group. However, some robots (such as the
:class:`RFL Robot<compas_fab.robots.RobotCellLibrary.rfl>`) have multiple 'arms',
where each arm can be planned to reach a target independently.
Even for a single-arm robot, planning groups can also be used to restrict the
planning backend to not use certain joints for planning, for example, to keep a
robot's linear track fixed.

**COMPAS FAB** uses the planning group's last link (end-effector link) to denote
the Planner Coordinate Frame (PCF), which is used when specifying targets for
motion planning. Any :class:`ToolModels<compas_robots.ToolModel>` that are attached
to the robot are attached to the PCF of the planning group as specified in the
:class:`ToolState.attached_to_group<compas_fab.robots.ToolState>`.

Usage
=====

.. currentmodule:: compas_fab.backends

When using the ROS Planning Backend, the planning groups are defined in the
`.srdf` (Semantic Robot Description Format) as part of the robot's MoveIt! package.
In this case, the planning groups cannot be changed by the user after MoveIt! is
launched. The user can only choose which planning group to use when calling the
planning functions.

When using the PyBullet Planner, the planning groups can be changed by the user
before :meth:`PyBulletPlanner.set_robot_cell` is called.

.. currentmodule:: compas_fab.backends
The planning group name is used as input to many planning functions, such as
:meth:`compas_fab.backends.interfaces.PlannerInterface.plan_motion` and
:meth:`compas_fab.backends.interfaces.PlannerInterface.inverse_kinematics`.

Examples
========
The following example prints out all the planning group names, base links, and
end-effector links for the RFL Robot.
The RFL Robot contains many planning groups for each of the four arms.

.. literalinclude:: files/01_planning_group_names.py
   :language: python

The following example prints out the only planning group for the ur10e robot.
The list of Links and Joints are printed.

.. literalinclude:: files/01_planning_group_print.py
   :language: python

Notes
=====

**COMPAS FAB** only supports planning groups that contain a serial chain of
joints, where all links in the group are connected by one continuous chain of
joints.