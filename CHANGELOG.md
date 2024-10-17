# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Added

* Added `Robot.from_urdf()` method to load a robot from a URDF file, can optionally take SRDF and meshes.
* Added unit test for `PyBulletClient` and `PyBulletPlanner` backend features, including Ik and FK agreement tests.
* Added `PyBulletClient.load_existing_robot` to load from an existing Robot, such as those in RobotLibrary.
* Added `compas_fab.robots.Waypoints` class to represent a sequence of targets. It has two child classes: `FrameWaypoints` and `PointAxisWaypoints`.
* Added `compas_fab.robots.Target` class to represent a motion planning target.
* Added also child classes `FrameTarget`, `PointAxisTarget`, `ConfigurationTarget`, `ConstraintSetTarget`
* Unlike previous constraints, `Targets` do not contain `group` as parameter. Instead, group parameter is passed to the planning call.
* Target scaling function is now embeded in the code for Targets. `scaled()` should be called by the user before passing the target to the `plan_motion` function.

### Changed

* Dropped support for Python 3.8 and updated compas requirements to 2.3
* Calling `forward_kinematics` from the Robot class now uses only the RobotModel to calculate the forward kinematics.
* Fixed error in `PyBulletForwardKinematics.forward_kinematics` where function would crash if `options` was not passed.
* Fixed error in `PyBulletInverseKinematics._accurate_inverse_kinematics` where threshold was not squared for comparison.
* Renamed `PybulletClient.get_cached_robot` to `PybulletClient.get_cached_robot_model` to avoid confusion between the `RobotModel` and `Robot` class.
* Renamed `PybulletClient.ensure_cached_robot` to `PybulletClient.ensure_cached_robot_model`.
* Renamed `PybulletClient.ensure_cached_robot_geometry` to `PybulletClient.ensure_cached_robot_model_geometry`.
* Renamed `PybulletClient.cache_robot` to `PybulletClient.cache_robot_model`.
* Backend planners use multi-inherence instead of `__call__` to include the backend functions. This allows for better generated documentation.
* `Robot.plan_cartesian_motion()` now accepts `Waypoints` as target. Implementation for `FrameWaypoints` is supported with same functionality as before. Simply wrap `Frame` objects using `FrameWaypoints(frames)`.
* Changed `BoundingVolume`, `Constraint`, `JointConstraint`, `OrientationConstraint`, `PositionConstraint` to inherit from `compas.data.Data` class.
* Change the signature of `plan_motion()` to use `target` (`Target` class) instead of `goal_constraints`. Only one target is accepted. Users who wish to compose their own constraint sets can still use `ConstraintSetTarget`.
* Moved `Robot.orientation_constraint_from_frame()` to `OrientationConstraint.from_frame()`, as constraints are no longer intended for users to use directly.
* Moved `Robot.position_constraint_from_frame()` to `PositionConstraint.from_frame()`, as constraints are no longer intended for users to use directly.
* Moved `Robot.constraints_from_frame()` to ros.backend_features and is handled by `convert_target_to_goal_constraints()`. Users who wish to use a frame as target should use a `FrameTarget` instead.
* Changed the behavior of Duration class when accepting both seconds (float) and nanoseconds (int) where the decimal point of seconds and the nanoseconds add up to more than 1 second.
* Changed GH Component `ConstraintsFromPlane` to `FrameTargetFromPlane`.
* Changed GH Component `ConstraintsFromTargetConfiguration` to `ConfigurationTarget`.

### Removed

* Removed `Tool` class from `compas_fab.robots` module.
* Removed Backend Feature `AddCollisionMesh`, `AppendCollisionMesh`, `AddAttachedCollisionMesh`, `RemoveCollisionMesh` and `RemoveAttachedCollisionMesh`.
* Removed `inverse_kinematics`, `forward_kinematics`, `plan_cartesian_motion`, and `plan_motion` methods from ClientInterface, access them using the planner instead.
* Removed `inverse_kinematics`, `plan_cartesian_motion`, and `plan_motion` methods from Robot, access them using the planner instead.
* Removed `Robot.ensure_client` method. Client and planner now exist independently.
* Removed `Robot.client` attribute. Access the planner functions directly using the planner instead.
* Removed `plan_cartesian_motion_deprecated` and `plan_motion_deprecated` methods from `Robot` class
* Removed `forward_kinematics_deprecated` and `inverse_kinematics_deprecated` method from `Robot` class

## [1.0.2] 2024-02-22

### Added

### Changed

* Raise `BackendFeatureNotSupportedError` exceptions when a features is not supported by the planner, instead of generic `Exception`.

### Removed


## [1.0.1] 2024-02-20

### Added

### Changed

### Removed


## [1.0.0] 2024-02-20

### Added

* Add parameter to control which link the tool is connected to in Grasshopper.
* Introduced `compas_fab.robots.RobotLibrary` class with 4 built-in robots.
* Added a script to extract URDF, SRDF and meshes from a Docker MoveIt instance.

### Changed

* Changed `CollisionMesh` inherit from `compas.data.Data`
* Changed `AttachedCollisionMesh` to inherit from `compas.data.Data`
* Changed `Robot` inherit from `compas.data.Data`
* Changed `RobotSemantics` inherit from `compas.data.Data`
* Changed `Tool` to inherit from `compas.data.Data`
* Renamed `Tool.link_name` to `Tool.connected_to`
* Migrate to COMPAS 2.x: add dependency to `compas_robots`
* Migrate to COMPAS 2.x: use `compas.tolerance` module instead of `compas.PRECISION`
* Add `attributes` to `Trajectory` class.
* Fixed `data` serialization API to comply with `COMPAS 2.0` private data API.
* Use the tool's `connected_to` link when showing end-effector frames in Grasshopper.
* Change default end-effector link name from `ee_link` to `tool0`.

### Removed

* Removed V-Rep backend.
* Removed outdated `PathPlan` class.
* Removed outdated rfl demo class.
* Remove deprecated aliases for artists (currently on `compas_robots`).
* Removed `compas_fab.robots.ur5` because it is now part of `compas_fab.robots.RobotLibrary`.
* Removed data files of ur5 and ur10e from `src/compas_fab/data/universal_robots` because they are now in of `src/compas_fab/data/robot_library`.

## [0.28.0] 2023-05-10

### Added

* Added `Forward Kinematics` GH component.

### Changed

* Updated install process of GH components.
* Added caching to the GH component that visualizes scene, to avoid retrieving the whole scene too often.

### Fixed

* Fixed pre-Noetic support on the MoveIt planner when a tool is attached to the robot.

## 0.27.0

### Added

* Added support for attached and non-attached collision mesh visualization to the `Robot Visualize` GH component.
* Added a prefix to all GH components.
* Added `append` to the operations of the `Collision Mesh` GH component.

### Changed

* Changed behavior of `Attach Tool` GH component to only attach the tool but not add it to the planning scene state.
* Duration class takes floats as `sec` variable.
* Changed the behavior of `forward_kinematics`, `inverse_kinematics`, `iter_inverse_kinematics`, `plan_cartesian_motion` and constraints construction methods (`orientation_constraint_from_frame`, `position_constraint_from_frame`, `constraints_from_frame`) in `Robot` class to use the frame of the attached tool if a tool is attached. This behavior can be reverted back (ie. only calculate T0CF) using the flag `use_attached_tool_frame` of all these methods.
* Fixed usage of `tangent_points_to_circle_xy` in Spherical Wrist solver to work with COMPAS v1.16 and older.

### Fixed

* Fixed DH params for analytical IK solver of UR3e and UR10e.
* Fixed Kinetic support on IK, FK, and motion planning calls.
* Fixed `Publish to topic` Grasshopper component when the `ros_client` has been replaced (eg. disconnected and reconnected).

### Deprecated

### Removed

## 0.26.0

### Added

* Added a new GH component - `ConstraintsFromTargetConfiguration`
* Added some missing information to GH and V-REP docs.
* Added a `Robot().attached_tools` property to allow attaching tools to multiple planning groups simultaneously.

### Changed

* Replaced icon for GH component - `ConstraintsFromPlane`
* `Robot().attached_tool` now points to the tool attached to the `robot.main_group_name`.
* Added parameter `group` to the `AttachToolComponent`

### Fixed

* Attaching a tool to a planning group doesn't overwrite the tool attached to other groups.
* Changed `Trajectory` to inherit from `compas.data.Data` class to fix a serialization error that expects guid to be present.

### Deprecated

### Removed

## 0.25.0

### Changed

* Changed Grasshopper components to default to icon display.
* Changed to use `compas_rhino.conversions` to coerce frames.

### Fixed

* Fixed link parameter name when doing FK inside the GH component to display attached collision meshes.
* Fixed transform of the attached collision mesh frame inside the GH component.
* Fixed uninstall process not removing GH components.

## 0.24.0

### Added

* Added `compas_fab.robots.ReachabilityMap`
* Added `compas_fab.robots.DeviationVectorsGenerator`
* Added `compas_fab.robots.OrthonormalVectorsFromAxisGenerator`

### Changed

### Fixed

* Fixed `ROSmsg` import on GH components for publish/subscribe.

### Deprecated

### Removed

## 0.23.0

### Added

* Added `compas_fab.backends.PyBulletClient.load_ur5()` method to simplify some examples.
* Added Grasshopper components to get a zero configuration and to merge two configurations.

### Changed

* Moved all public API classes in `compas_fab.backends` to second-level imports.
* Updated to COMPAS 1.14.
* Simplified call to remove an attached tool by also removing the remaining collision mesh from the world automatically.

### Fixed

* Fixed PyBullet loading of meshes.
* Fixed missing flag in reset planning scene call.
* Fixed issue on cartesian and kinematic planning when model contains passive joints.
* Fixed pose of collision mesh in ROS Noetic being ignored.

### Deprecated

* Deprecated `compas_fab.utilities.write_data_to_json` in favor of `compas.data.json_dump`.
* Deprecated `compas_fab.utilities.read_data_from_json` in favor of `compas.data.json_load`.

### Removed

## 0.22.0

### Added

* Added `Attach Tool` GH component: crowd-coded at McNeel's Robotic Fabrication Workshop!

### Changed

### Fixed

### Deprecated

### Removed

## 0.21.1

### Added

### Changed

* Changed default wire visibility to hidden in some GH components for cleaner Grasshopper files.

### Fixed

### Deprecated

### Removed

## 0.21.0

### Added

* Added a new backend: analytical kinematics for spherical-wrist and offset-wrist robots.

### Fixed

* Consider `AttachedCollisionMesh` in `AnalyticalInverseKinematics`.

## 0.20.1

### Removed

* Removed the bundled binary files for the `VrepClient` remote API. To use V-REP, use the `remoteApi` binaries provided with the software.

## 0.20.0

### Added

* Added `PoseArray`, `MultiArrayDimension`, `MultiArrayLayout`, `Int8MultiArray`, `Float32MultiArray`, `Int32` to `compas_fab.backends.ros.messages`
* Added `unordered_disabled_collisions` attribute to `PyBulletClient` and `RobotSemantics`
* Added better support for concave meshes in the `PyBulletClient`
* Added `Robot.iter_inverse_kinematics` to allow iterating over all IK solutions provided by a solver

### Changed

* Changed the backend feature `InverseKinematics.inverse_kinematics` to be a generator. As a consequence of this, `ClientInterface.inverse_kinematics` and `PlannerInterface.inverse_kinematics` have changed to generators as well
* Standardized the yielded type of `InverseKinematics.inverse_kinematics` across the PyBullet, MoveIt and V-REP planners
* Added iterative accurate IK resolution for PyBullet

### Fixed

* Fixed `UnsupportedOperation` error when using `PyBulletClient` in Jupyter notebook (raised by `redirect_stdout`)
* Fixed `JointTrajectoryPoint.from_data` to be backward-compatible with JSON data generated before `compas_fab` 0.18
* Fixed `JointTrajectory.from_data` to be backward-compatible with JSON data generated before `compas_fab` 0.17

### Deprecated

### Removed

## 0.19.1

### Added

### Changed

### Fixed

* Fixed bundling of ghuser components

### Deprecated

### Removed

## 0.19.0

### Added

* Added documentation for Grasshopper components.
* Added Grasshopper components to publish and subscribe to ROS topics.

### Changed

* Updated `build-ghuser-components` task
* Updated to COMPAS 1.7

### Fixed

### Deprecated

### Removed

## 0.18.3

### Added

### Changed

* Made consistent use of `repr` in nested objects

### Fixed

* Fixed bug in `compas.backends.PyBulletClient.convert_mesh_to_body` circumventing PyBullet's propensity to cache

### Deprecated

### Removed

## 0.18.2

### Added

### Changed

### Fixed

### Deprecated

### Removed

## 0.18.1

### Fixed

* Fix error message during uninstall of Grasshopper components

## 0.18.0

### Added

* Grasshopper components now also for Mac
* Added support for MoveIt on ROS Noetic
* Added support for Python 3.9

### Changed

* The `Configuration` class has moved to `compas.robots`, but is still aliased within `compas_fab.robots`
* Lazily load `V-REP remoteApi` library

### Fixed

* Fixed `repr()` of `ROSmsg` class
* Fixed data type of secs and nsecs in `Time` ROS message
* Fixed `CollisionObject.to_collision_meshes`
* Fixed serialization of joint names for `compas_fab.robots.JointTrajectoryPoint`
* Fixed deserialization of `AttachedCollisionMesh`

### Deprecated

* `compas_fab.robots.Configuration` is being deprecated in favor of `compas.robots.Configuration`

## 0.17.0

### Added

* Added python components library for Grasshopper
* Added `compas_fab.robots.PyBulletClient.get_robot_configuration`
* Added `compas_fab.robots.Robot.ensure_geometry`
* Added serialization methods to `compas_fab.robots.CollisionMesh` and `compas_fab.robots.AttachedCollisionMesh`
* Added `attached_collision_meshes` attribute to `compas_fab.robots.JointTrajectory`
* Added `compas_fab.backends.PlanningSceneComponents.__ne__`
* Added dictionary behavior to `compas_fab.robots.JointTrajectoryPoint.merge`
* Added length limitations to attributes of `compas_fab.robots.JointTrajectoryPoint.merge`

### Changed

* Updated to `COMPAS 1.1`
* `Configuration` & `JointTrajectoryPoint`: the attributes `values` and `types` changed to `joint_values` and `joint_types` respectively.

### Fixed

* Fixed bug in the PyBullet client where one could not update the configuration of a robot with an attached collision mesh
* Fixed bug existing since version 0.12 where `compas_fab.backends.RosClient.add_attached_collision_mesh` added collision objects to the scene, but did not attached them to the robot
* Fixed bug when keys with `None` values were passed to the planner.

### Deprecated

### Removed

* Remove `compas_fab.robots.JointTrajectoryPoint.merge`

## 0.16.0

### Changed

* Updated to `COMPAS 1.0`

## 0.15.0

### Added

### Changed

* Updated to `COMPAS 0.19`

### Fixed

### Deprecated

### Removed

## 0.14.0

### Added

* Added new backend feature `ResetPlanningScene`
* Added `MoveItResetPlanningScene`

### Changed

* Updated to `COMPAS 0.18`
* Use `compas.IPY` to check for IronPython

### Fixed

* Fixed bug in `remove_attached_tool` of `PlanningScene`

## 0.13.1

### Added

* Added `name` property to `Tool` class.

### Fixed

* Fixed bug in `add_attached_tool` of `PlanningScene`
* Fixed `frame_id` generation when tool name changes
* Fixed freeze with some sync planning scene methods on Grasshopper/IronPython

## 0.13.0

### Changed

* Updated to `COMPAS 0.17`

## 0.12.0

### Added

* **PyBullet integration**: added support for PyBullet client and forward/inverse kinematic solver
* Added `ClientInterface`, `PlannerInterface` and various backend feature interfaces
* Added implementations of these interfaces for ROS and V-REP
* Added `attributes` dictionary to `Robot` class
* Added `compas_fab.robots.Tool.from_t0cf_to_tcf`
* Added `compas_fab.robots.Tool.from_tcf_to_t0cf`
* Added `joint_names` as optional parameter for all `compas_fab.robots.Configuration` constructors
* Added `compas_fab.robots.Configuration.iter_differences`
* Added `compas_fab.robots.Configuration.max_difference`
* Added `compas_fab.robots.Configuration.close_to`
* Added `compas_fab.robots.Configuration.merge`
* Added `compas_fab.robots.JointTrajectoryPoint.merge`
* Added `compas_fab.robots.Semantics.group_states`
* Added `compas_fab.robots.Robot.get_configuration_from_group_state`

### Changed

* Updated to `COMPAS 0.16.9`
* Renamed `compas_fab.robots.Robot.to_local_coords` to `compas_fab.robots.Robot.to_local_coordinates`
* Renamed `compas_fab.robots.Robot.to_world_coords` to `compas_fab.robots.Robot.to_world_coordinates`
* Backend clients have been restructured according to the new interfaces
* Parameter `backend` of forward kinematics has been renamed to `solver`
* The signatures of all kinematics, motion planning and planning scene management methods have been homogenized across backend clients and within `Robot`
* All examples have been updated to reflect these changes
* The installer to Rhino has been unified with COMPAS core. Now running `python -m compas_rhino.install` will also detect and install COMPAS FAB and its dependencies.
* Renamed all `RobotArtist` implementations to `RobotModelArtist` to reflect
  the fact they depend on `compas.robots.RobotModel`.
* Renamed  `compas_fab.robots.Robot.from_tool0_to_attached_tool` to `compas_fab.robots.Robot.from_t0cf_to_tcf`
* Renamed  `compas_fab.robots.Robot.from_attached_tool_to_tool0` to `compas_fab.robots.Robot.from_tcf_to_t0cf`
* Changed ROS planning scene methods to be synchronous.


### Fixed

* Attached collision meshes are included in inverse kinematics calculations in ROS

### Deprecated

* The methods `forward_kinematics`, `inverse_kinematics`, `plan_cartesian_motion` and `plan_motion`
  of `Robot` class have been refactored, but a backwards-compatible deprecated version with the old
  signatures still exists suffixed by `_deprecated`, e.g. `forward_kinematics_deprecated`.
* `RobotArtist` are deprecated in favor of `RobotModelArtist`.

### Removed

## 0.11.0

### Added

* Added optional `joint_names` to `Configuration`
* Added `Configuration.scaled`
* Added `full_joint_state` to `Robot.inverse_kinematics`
* Added `Semantics.get_all_configurable_joints`

### Changed

* Updated to `COMPAS 0.15`
* Construct `full_configuration` with `values`, `types`, `joint_names` in `Robot` rather than in `MoveItPlanner`
* `MoveItPlanner` returns `start_configuration` with set `joint_names`
* Removed parameter `names` from `RobotArtist.update`
* Updated Grasshopper examples
* `Robot`: `forward_kinematics` returns now `frame_WCF`
* `MoveItPlanner`: `forward_kinematics` takes now instance of `Configuration` and `robot`
* `MoveItPlanner`: `inverse_kinematics` takes now instance of `Configuration` and `robot`
* Property :class:`compas_fab.robots.Robot.artist` does not try to scale robot
  geometry if links and/or joints are not defined.
* In :class:`compas_fab.robots.constraints.JointConstraint`, added `tolerance_above` and
  `tolerance_below` for allowing asymmetrical constraints.
* In :class:`compas_fab.robots.Robot`, changed the `constraints_from_configuration`
  function with `tolerances_above` and `tolerances_below`.
* :meth:`compas_fab.robots.CollisionMesh.scale` now takes a scale factor
  instead of a :class:`compas.geometry.Scale` instance as an argument.

### Fixed

* Convert constraints on inverse kinematics and cartesian planner to ROS messages
* Fix support for trajectory constraints on kinematic planner

## 0.10.2

### Added

* Added Python 3.8 support

### Changed

* Updated to `COMPAS 0.13`

## 0.10.1

### Fixed

* Fix DAE parser to handle `polylist` meshes
* Bumped `roslibpy` dependency to `0.7.1` to fix blocking service call issue on Mac OS

## 0.10.0

### Added

* Added `attach_tool`, `detach_tool`, `draw_attached_tool`, `from_tool0_to_attached_tool` and `from_attached_tool_to_tool0` to `Robot`
* Added `attach_tool` and `detach_tool` to `Artist`
* Added `add_attached_tool` and `remove_attached_tool` to `PlanningScene`
* Added redraw/clear layer support to :class:`~compas_fab.rhino.RobotArtist` for Rhino
* Added material/color support for DAE files on ROS file loader

### Changed

* Changed `inverse_kinematics`, `plan_cartesian_motion` and `plan_motion` to use the attached_tool's `AttachedCollisionMesh` if set

### Fixed

* Fixed mutable init parameters of `Configuration`, `JointTrajectoryPoint`, `JointTrajectory` and `Robot.basic`.
* Fixed interface of :class:`~compas_fab.blender.RobotArtist` for Blender
* Fixed DAE parsing of meshes with multiple triangle sets

## 0.9.0

### Added

* Added `load_robot` method to ROS client to simplify loading robots from running ROS setup.
* Added `compas_fab.robots.Wrench`: a Wrench class representing force in free space, separated into its linear (force) and angular (torque) parts.
* Added `compas_fab.robots.Inertia`: a Inertia class representing spatial distribution of mass in a rigid body

### Changed

* Updated to `COMPAS 0.11`

## 0.8.0

### Changed

* Updated to `COMPAS 0.10`
* Add better support for passive joints on IK, Cartesian and Kinematic planning

### Fixed

* Use WorldXY's origin as default for robots that are have no parent join on their base
* Fixed parsing of semantics (SRDF) containing nested groups
* Fixed DAE support on ROS File loader

## 0.7.0

### Changed

* Fixed Python 2 vs Python 3 incompatibilities in `compas_fab.sensors` module
* Changed example for loading PosConCM (includes parity argument, differs from PosCon3D)
* Changed format `compas_fab.sensors.baumer.PosConCM.set_flex_mount()`
* Changed tasks.py to run `invoke test`
* Renamed `compas_fab.backends.CancellableTask` to `compas_fab.backends.CancellableFutureResult`
* ROS client: changed joint trajectory follower (`follow_joint_trajectory`) to support generic `JointTrajectory` arguments.
* ROS client: changed return type of trajectory execution methods to `CancellableFutureResult`

### Added

* Added `compas_fab.sensors.baumer.PosCon3D.reset()`
* Added `compas_fab.sensors.baumer.PosConCM.reset()`
* ROS client: added support for MoveIt! execution action via `client.execute_joint_trajectory`.
* Added `compas_fab.backends.FutureResult` class to deal with long-running async tasks

### Removed

* Removed `compas_fab.sensors.baumer.PosConCM.get_live_monitor_data()`
* Removed non-implemented methods from `compas_fab.robots.Robot`: `send_frame`, `send_configuration`, `send_trajectory`

### Fixed

* Fixed missing planner initialization when used without context manager.

## 0.6.0

### Changed

* Updated `COMPAS` dependency to `0.8.1`
* Base robot artist functionality moved to `compas.robots.RobotModel`
* `Robot`: `inverse_kinematics` returns now group configuration
* `Robot`: `forward_kinematics` has new parameter `backend` to select either `client` FK or `model` FK.
* `Robot`: `forward_kinematics` returns now `frame_RCF`
* `Robot`: `forward_kinematics` doesn't need full configuration anymore
* Fixed delays when modifying the planning scene of ROS.

### Added

* Added `jump_threshold` parameter to `plan_cartesian_motion`
* Added `action_name` parameter to reconfigure joint trajectory follower action.
* Added support to retrieve the full planning scene.

### Removed

* Removed `compas_fab.Robot.get_configuration`

## 0.5.0

### Changed

* ROS Client: renamed `compute_cartesian_path` to `plan_cartesian_motion`
* ROS Client: renamed `motion_plan_goal_frame` and
  `motion_plan_goal_configuration` to `plan_motion`
* ROS Client: removed methods from `Robot` that are now handled with
  `PlanningScene`, e.g. `add_collision_mesh` and
  `add_attached_collision_mesh`
* ROS Client: change the return type of `plan_motion` and `plan_cartesian_motion`
  to the new trajectory classes.
* ROS File Server Loader: moved to `compas_fab.backends` package
* ROS File Server Loader: renamed `load` to `load_urdf` and sync'd API to other loaders.
* V-REP Client: renamed `get_end_effector_pose` to `forward_kinematics`
* V-REP Client: renamed `find_robot_states` to `inverse_kinematics`
* V-REP Client: renamed `find_path_plan_to_config` to
  `plan_motion_to_config`
* V-REP Client: renamed `find_path_plan` to `plan_motion`
* V-REP Client: changed `is_connected` to become a property
* Made `robot_artist` default `None` on `Robot` constructor
* Changed `PathPlan` class to use the new trajectory classes

### Added

* Added `scale` method to `Configuration`
* Implemented Constraints (`OrientationConstraint`, `PositionConstraint`, `JointConstraint`) to use with `plan_motion`
* Implemented `PlanningScene`, `CollisionMesh` and `AttachedCollisionMesh`
* Added generic representations for motion planning requests (`JointTrajectory`, `JointTrajectoryPoint`, `Duration`)
* Added UR5 robot model data for example purposes
* Added several doc examples

### Removed

* Aliases for `Frame` and `Transformation`. Import from `compas.geometry` instead.

## 0.4.1

### Fixed

* Fixed missing library for V-REP on macOS

### Deprecated

* The aliases for `Frame` and `Transformation` will be removed, in the future, import directly from `compas` core.

## 0.4.0

### Added

* Color parameter to Rhino robot artist

### Changed

* Updated to `COMPAS 0.4.10`

## 0.3.0

### Added

* Deeper integration with MoveIt! motion planning services
* Added sync and async versions of many ROS service calls
* Added support for cancellable tasks/actions

### Changed

* Renamed `UrdfImporter` to `RosFileServerLoader`
* Updated to `COMPAS 0.4.8`

## 0.2.1

### Added

* Robot artist for Blender

## 0.2.0

### Added

* First open source release!
* V-REP and ROS clients
* Updated to `COMPAS 0.3.2`

## 0.1.0

### Added

* Initial version
