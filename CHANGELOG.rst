
Changelog
=========

All notable changes to this project will be documented in this file.

The format is based on `Keep a Changelog <https://keepachangelog.com/en/1.0.0/>`_
and this project adheres to `Semantic Versioning <https://semver.org/spec/v2.0.0.html>`_.

Unreleased
----------

**Added**

* Added ``attach_tool``, ``detach_tool``, ``draw_attached_tool``, ``from_tool0_to_attached_tool`` and ``from_attached_tool_to_tool0`` to ``Robot``
* Added ``attach_tool`` and ``detach_tool`` to ``Artist``
* Added ``add_attached_tool`` and ``remove_attached_tool`` to ``PlanningScene``
* Added redraw/clear layer support to `RobotArtist` for Rhino
* Added material/color support for DAE files on ROS file loader

**Changed**

* Changed ``inverse_kinematics``, ``plan_cartesian_motion`` and ``plan_motion`` to use the attached_tool's ``AttachedCollisionMesh`` if set

**Removed**

**Fixed**

* Fixed mutable init parameters of ``Configuration``, ``JointTrajectoryPoint``, ``JointTrajectory`` and ``Robot.basic``.
* Fixed interface of `RobotArtist` for Blender

**Deprecated**

0.9.0
----------

**Added**

* Added ``load_robot`` method to ROS client to simplify loading robots from running ROS setup.
* Added ``compas_fab.robots.Wrench``: a Wrench class representing force in free space, separated into its linear (force) and angular (torque) parts.
* Added ``compas_fab.robots.Inertia``: a Inertia class representing spatial distribution of mass in a rigid body

**Changed**

* Updated to COMPAS 0.11

0.8.0
----------

**Changed**

* Updated to COMPAS 0.10
* Add better support for passive joints on IK, Cartesian and Kinematic planning

**Fixed**

* Use WorldXY's origin as default for robots that are have no parent join on their base
* Fixed parsing of semantics (SRDF) containing nested groups
* Fixed DAE support on ROS File loader

0.7.0
----------

**Changed**

* Fixed Python 2 vs Python 3 incompatibilities in ``compas_fab.sensors`` module
* Changed example for loading PosConCM (includes parity argument, differs from PosCon3D)
* Changed format ``compas_fab.sensors.baumer.PosConCM.set_flex_mount()``
* Changed tasks.py to run ``invoke test``
* Renamed ``compas_fab.backends.CancellableTask`` to ``compas_fab.backends.CancellableFutureResult``
* ROS client: changed joint trajectory follower (``follow_joint_trajectory``) to support generic ``JointTrajectory`` arguments.
* ROS client: changed return type of trajectory execution methods to ``CancellableFutureResult``

**Added**

* Added ``compas_fab.sensors.baumer.PosCon3D.reset()``
* Added ``compas_fab.sensors.baumer.PosConCM.reset()``
* ROS client: added support for MoveIt! execution action via ``client.execute_joint_trajectory``.
* Added ``compas_fab.backends.FutureResult`` class to deal with long-running async tasks

**Removed**

* Removed ``compas_fab.sensors.baumer.PosConCM.get_live_monitor_data()``
* Removed non-implemented methods from ``compas_fab.robots.Robot``: ``send_frame``, ``send_configuration``, ``send_trajectory``

**Fixed**

* Fixed missing planner initialization when used without context manager.

0.6.0
----------

**Changed**

* Updated ``COMPAS`` dependency to ``0.8.1``
* Base robot artist functionality moved to ``compas.robots.RobotModel``
* ``Robot``: ``inverse_kinematics`` returns now group configuration
* ``Robot``: ``forward_kinematics`` has new parameter ``backend`` to select either ``client`` FK or ``model`` FK.
* ``Robot``: ``forward_kinematics`` returns now ``frame_RCF``
* ``Robot``: ``forward_kinematics`` doesn't need full configuration anymore
* Fixed delays when modifying the planning scene of ROS.

**Added**

* Added ``jump_threshold`` parameter to ``plan_cartesian_motion``
* Added ``action_name`` parameter to reconfigure joint trajectory follower action.
* Added support to retrieve the full planning scene.

**Removed**

* Removed ``compas_fab.Robot.get_configuration``

0.5.0
----------

**Changed**

* ROS Client: renamed ``compute_cartesian_path`` to ``plan_cartesian_motion``
* ROS Client: renamed ``motion_plan_goal_frame`` and
  ``motion_plan_goal_configuration`` to ``plan_motion``
* ROS Client: removed methods from ``Robot`` that are now handled with
  ``PlanningScene``, e.g. ``add_collision_mesh`` and
  ``add_attached_collision_mesh``
* ROS Client: change the return type of ``plan_motion`` and ``plan_cartesian_motion``
  to the new trajectory classes.
* ROS File Server Loader: moved to ``compas_fab.backends`` package
* ROS File Server Loader: renamed ``load`` to ``load_urdf`` and sync'd API to other loaders.
* V-REP Client: renamed ``get_end_effector_pose`` to ``forward_kinematics``
* V-REP Client: renamed ``find_robot_states`` to ``inverse_kinematics``
* V-REP Client: renamed ``find_path_plan_to_config`` to
  ``plan_motion_to_config``
* V-REP Client: renamed ``find_path_plan`` to ``plan_motion``
* V-REP Client: changed ``is_connected`` to become a property
* Made ``robot_artist`` default ``None`` on ``Robot`` constructor
* Changed ``PathPlan`` class to use the new trajectory classes

**Added**

* Added ``scale`` method to ``Configuration``
* Implemented Constraints (``OrientationConstraint``, ``PositionConstraint``, ``JointConstraint``) to use with ``plan_motion``
* Implemented ``PlanningScene``, ``CollisionMesh`` and ``AttachedCollisionMesh``
* Added generic representations for motion planning requests (``JointTrajectory``, ``JointTrajectoryPoint``, ``Duration``)
* Added UR5 robot model data for example purposes
* Added several doc examples

**Removed**

* Aliases for ``Frame`` and ``Transformation``. Import from ``compas.geometry`` instead.

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
