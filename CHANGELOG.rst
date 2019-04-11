
Changelog
=========

All notable changes to this project will be documented in this file.

The format is based on `Keep a Changelog <http://keepachangelog.com/en/1.0.0/>`_
and this project adheres to `Semantic Versioning <http://semver.org/spec/v2.0.0.html>`_.

Unreleased
----------

**Fixed**

**Changed**

* Renamed ``compute_cartesian_path`` to ``plan_cartesian_motion``
* Renamed ``motion_plan_goal_frame`` and ``motion_plan_goal_configuration`` to ``plan_motion``
* Removed methods from ``Robot`` that are now handled with ``PlanningScene``, e.g. ``add_collision_mesh`` and ``add_attached_collision_mesh``
* Made ``robot_artist`` default ``None`` on ``Robot`` constructor

**Added**

* Implemented Constraints (``OrientationConstraint``, ``PositionConstraint``, ``JointConstraint``) to use with ``plan_motion``
* Implemented ``PlanningScene``, ``CollisionMesh`` and ``AttachedCollisionMesh`` 
* Added UR5 robot model data for example purposes
* Added several doc examples

**Deprecated**


0.4.1
----------

**Fixed**

* Fixed missing library for V-REP on macOS

**Deprecated**

* The aliases for ``Frame`` and ``Transformation`` will be removed, in the future, import directly from `compas` core.

0.4.0
----------

**Added**

* Color parameter to Rhino robot artist

**Changed**

* Updated to COMPAS 0.4.10

0.3.0
----------

**Added**

* Deeper integration with MoveIt! motion planning services
* Added sync and async versions of many ROS service calls
* Added support for cancellable tasks/actions

**Changed**

* Renamed ``UrdfImporter`` to ``RosFileServerLoader``
* Updated to COMPAS 0.4.8

0.2.1
----------

**Added**

* Robot artist for Blender

0.2.0
-----

**Added**

* First open source release!
* V-REP and ROS clients
* Updated to COMPAS 0.3.2

0.1.0
-----

**Added**

* Initial version
