.. _rigid_body:

********************************************************************************
Rigid Body
********************************************************************************

A RigidBody is a physical object in the robot cell that can be stationary or
attached to the robot. The :class:`RigidBody` class in **COMPAS FAB** is used
to represent the geometry of these objects while the :class:`RigidBodyState`
class is used to represent the pose, attachment relationship and
allowable-collision relationship of the rigid body.

The :class:`RigidBody.visual_meshes` and the :class:`RigidBody.collision_meshes`
attributes are used to represent the visual and collision geometry of the rigid
body, respectively. Both attributes are optional, but typically at least one of
them is required. The visual geometry is used for visualization in CAD software,
if left empty, no visualization will be shown. The collision geometry is used for
collision checking, if left empty, the rigid body will not be considered in
collision checking.
The :class:`RigidBody.native_scale` attribute can be used denote the scale of the
meshes. See :ref:`Core Concepts: Native Scale<native_scale>` for more information.

.. _rigid_body_state:

Rigid Body State
================

The :class:`RigidBodyState` class is used to represent a number of properties that
can be changed from one instance to another. This is not limited to the pose of the
rigid body (:attr:`RigidBodyState.pose`), but also includes the attachment
relationship of the rigid body to the robot. (:attr:`RigidBodyState.attached_to_link`
and :attr:`RigidBodyState.attached_to_tool`). There are three possible attachment
scenarios:

**Stationary**: The rigid body is not attached to the robot and is stationary in the
robot cell. For example, a table or a wall that contributes to the robot cell's
collision environment. In this case, the ``.attached_to_link`` and ``.attached_to_tool``
attributes are set to ``None`` (default).

**Attached to Link**: The rigid body is attached to a specific link of the robot.
This is useful for representing cable dress packs and tool components that are
often attached to the 'upper arm' of the robot. In this case, the ``.attached_to_link``
attribute is set to the name of the link. The ``.attachment_frame`` attribute
represent the frame of the rigid body (OCF) relative to the link (LCF).

**Attached to Tool**: The rigid body is attached to the robot's tool. This is useful
for representing workpieces that are attached to the robot via an attached tool.
In this case, the ``.attached_to_tool`` attribute is set to the name of the tool.
The ``.attachment_frame`` attribute represent the frame of the rigid body (OCF)
relative to the tool (TCF).

Usage
=====

The :class:`RigidBody` objects are used to compose the :class:`RobotCell`, which are
then passed to planning backends (:meth:`PlannerInterface.set_robot_cell`) or used
in visualization (:meth:`compas.scene.Scene.add`).

The :class:`RigidBodyState` objects are used to compose the :class:`RobotCellState`,
which are used in planning functions (such as :meth:`PlannerInterface.inverse_kinematics`)
and also in visualization (:meth:`BaseRobotCellObject.update`).