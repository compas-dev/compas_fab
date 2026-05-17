# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Added

* New backend page [Analytical IK + PyBullet](docs/backends/analytical_pybullet.md) for `AnalyticalPyBulletPlanner` — the hybrid planner that pairs closed-form analytical IK with PyBullet collision checking. Previously this backend was undocumented despite being shipped in `compas_fab.backends`.

### Changed

* Standardized every backend page (`docs/backends/{analytical,analytical_pybullet,pybullet,ros,ros2}.md`) to a common structure: *When to use → Trade-offs → Setup → First example (embedded) → More examples (linked) → API reference*. Each page now embeds one runnable example from `docs/backends/*/files/` via `pymdownx.snippets` and links the remaining ~50 examples to GitHub.
* Rewrote `docs/backends/index.md` as a real decision guide: a "by intent" table keyed by what the user wants to do (rather than backend name), plus a "by capability" matrix and explicit setup-cost summary. Linked from both Home and Installation.
* Split `docs/installation.md` into a focused library-install page (uv/pip/conda + verify) and a new `docs/frontends.md` covering CAD environment setup (Rhino 8, Grasshopper, Blender, COMPAS Viewer, headless). The new front-ends page also documents which CAD environments work with which backends, surfacing the limitation that PyBullet cannot run inside Rhino's interpreter.
* Rebranded the old `tutorial.md` as `concepts.md`: a backend-agnostic walkthrough of the data model (`RobotCell`, `RobotCellState`, `Target`/`Waypoints`, `TargetMode`). No planner calls — the concepts page now reads as "this is the data; backends are how you execute it" and links out to the backends section for the actual planning calls.
* Rewrote `docs/index.md` as a real landing page (was a 1-paragraph blurb): leads with the five-backend table, "I want to..." quick links, and a four-step what's-next list. Updated MkDocs nav to surface the new structure (Home → Installation → CAD front-ends → Concepts → Backends → API → Developer guide).

### Fixed

* `AnalyticalInverseKinematics.iter_inverse_kinematics` and its `_iter_inverse_kinematics_frame_target` helper had `group: Optional[str]` declared without a default, while the parent `InverseKinematics` interface and all example usages treat `group` as optional. Added `= None` so calls like `planner.iter_inverse_kinematics(target, start_state)` work as documented.
* Cleared stale docstring parameters across `compas_fab.backends` interfaces and backend feature implementations (`robot`, `client`, `solver`, `planner_type`) that no longer matched their signatures, plus a confusingly-indented continuation line in `RigidBodyState.attached_to_link`. `invoke docs` now builds with zero warnings.

### Changed

* Bumped `compas_robots` requirement from `>=0.6,<1` to `>=1,<2`.
* Migrated packaging to `pyproject.toml`.
* Cleaned up `requirements-dev.txt`.
* Python classifiers trimmed to 3.9 (minimum, required for Rhino 8) and 3.13 (latest).
* Unpinned `roslibpy` from `1.8.1` to `>=2.0,<3` in preparation for ROS 2 support.
* Updated `ActionClient` and `Goal` imports from `roslibpy.actionlib` to `roslibpy.ros1.actionlib` following the breaking namespace change in `roslibpy` 2.x.

### Removed

* All Sphinx-era documentation sources: every `docs/**/*.rst`, the `docs/conf.py` Sphinx config, the `docs/_static/` and `docs/_images/` asset folders, the `docs/developer/generated/` autosummary output, `docs/spelling_wordlist.txt` and `docs/doc_versions.txt` (the latter superseded by `mike`'s gh-pages versioning).
* `.. autosummary::`, `.. toctree::` and `.. currentmodule::` directives from all module-level `__init__.py` docstrings (replaced with prose + intra-doc Markdown links resolved by `mkdocstrings`).
* Removed `DirectUrActionClient` and associated `direct_ur` messages module since this UR-specific action client was outdated and unused.

### Added

* New `ros2-ur10e-demo` docker reference backend (ROS 2 Jazzy + MoveIt 2 + UR10e) with a `ur-sim` service running the official `universalrobots/ursim_e-series` Polyscope simulator, a `zenoh-router` service running `rmw_zenohd` as the RMW transport (replaces DDS — no more multicast discovery issues on Docker Desktop), a `ur-driver` service running the real `ur_robot_driver` against the simulator (with the ros2_control + RTDE update rate lowered from the default 500 Hz to 100 Hz via a baked-in `update_rate.yaml`, to stop the controller manager from spamming "Overrun detected!" warnings under Docker virtualisation), `moveit-demo` running `ur_moveit.launch.py`, rosbridge on `9090`, an HTTP file server on `9091` for serving meshes from `/opt/ros/jazzy/share/`, and a `theasp/novnc` web GUI on `8080` for viewing RViz.
* New `compas_fab.backends.HttpFileServerLoader` that mirrors `RosFileServerLoader`'s interface but fetches meshes over plain HTTP and reads URDF/SRDF from rosbridge topics (the ROS 2 convention) instead of ROS parameters.
* Migrated documentation from Sphinx to MkDocs Material, matching the structure used in `compas_robots`. New `mkdocs.yml` at the repository root; documentation sources are now Markdown (`docs/*.md`) with API pages driven by `mkdocstrings`. Backlinks are disabled. `inventories` includes `compas`, `compas_robots`, and `compas_viewer`. Sphinx config (`docs/conf.py`) and Sphinx-only `docs/requirements.txt` have been removed; `tasks.py` now invokes `compas_invocations2.mkdocs.docs` so `invoke docs` builds the MkDocs site.
* New developer guide section: [Backend architecture](docs/developer/architecture.md) ported from the old `docs/developer/backends.rst`, plus [Grasshopper components](docs/developer/grasshopper.md) ported from `docs/developer/grasshopper.rst`.
* New API pages explicitly exposing `compas_fab.backends.interfaces`, `compas_fab.backends.ros.backend_features` and `compas_fab.backends.pybullet.backend_features` via `mkdocstrings`, replacing the Sphinx `autosummary` tables that previously generated stub `.rst` files per symbol.
* New `Target` and `Waypoints` classes to represent inputs of planning functions.
  * New `PointAxisTarget` and `PointAxisWaypoint` classes for processes that have a cylindrical tool (e.g. drilling, milling, 3D printing).
* New `TargetMode` enum to specify how planning functions interpret a target frame.
  * New `TargetMode.Tool` mode allow users to specify the target location of the attached tool.
  * New `TargetMode.WORKPIECE` mode allow the same for the attached workpiece.
  * Forward kinematics functions also support target mode to return Robot, attached Tool or attached Workpiece frame.
* New `RobotCell` class to represent all objects in a a robot cells, namely the RobotModel, ToolModel(s) and RigidBody(s). RobotSemantics is also here. Replaced `Robot` class.
  * New `PlannerInterface.set_robot_cell` function for the user to pass the robot cell to the planner.
  * New `RigidBody` class support separated Visual and Collision geometries.
* New `RobotCellState` class to represent state of all objects in the robot cell, including robot Configuration, robot base frame (new), ToolState(s) and RigidBodyState(s).
  * New `RobotCellState._robot_base_frame` (relative to WCF) can now be changed by user.
    * Note that all RigidBody frames and Target frames are still relative to WCF (conceptually the same as before).
  * New `ToolState` class includes tool configuration (for kinematic tools) and attachment state (group and attachment frame) of the tool to the robot.
  * New `ToolState.attachment_frame` allows users to change the attachment location of a tool relative to the end-effector link.
  * New `RigidBodyState` class can model both options of attaching the object to a robot link (e.g. cable dress-pack) and attaching to the tool (which is now referred to as a Workpiece).
* Planning backends are redesigned to have stateless planning functions (e.g. `collision_check`, `inverse_kinematics`, `plan_motion`).
  * Stateless planning functions requires `RobotCellState` input, allowing different object states between planning calls.
  * It avoids the need for the user to keep track of the state in the planner between planning calls and even sessions.
  * It allows cleaner implementation when batch planning multiple motions with different attachment relationships.
* New functions for `PyBulletPlanner`.
  * New `PlanCartesianMotion` function for `PyBulletPlanner`. Compatible with `FrameWaypoints` and `PointAxisWaypoints`.
  * New `InverseKinematics` support for random restarts.
  * New `CollisionCheck` function for `PyBulletPlanner`.
* New support for `ToolModels` with kinematic joints.
  * Supported by both `PyBulletPlanner` and `MoveItPlanner`.
  * Allows users to model tools with moving parts (e.g. grippers jaws) for more accurate collision checking.
* New `RobotCellLibrary`, `ToolLibrary` and `RigidBodyLibrary` classes to quickly load pre-defined objects for tests, examples and demos.
  * New `RobotCellLibrary` contains pre-defined robots (e.g. UR5, UR10e, Panda, ABB IRB120, ABB IRB4600 and RFL) and also cells with tools and rigid bodies.
  * New `ToolLibrary` contain examples of static and kinematic tools.
  * New script to extract robot packages (URDF, SRDF and meshes) from the ROS docker to create new `RobotCellLibrary` objects.
* New `SceneObject` classes for visualization
  * Users should use the `RobotCellObject` class, which can visualize the entire robot cell.
  * Users should first call `.update()` with the state class, then call `.draw()` to draw native CAD geometries.
  * A temporary `RobotModelObject` class is provided to avoid changing the existing one in `compas_robots`
* Redesigned the mechanism for dealing with non-meter scale input and output.
  * Input: User created `RigidBody` can contain meshes that are not in meters by setting the `.native_scale` attribute.
    * `RigidBody.visual_meshes_in_meters()` and `RigidBody.collision_meshes_in_meters()` functions can return the meshes in meters.
  * Input: User created `ToolModel` must be in meters to align with `compas_robot.RobotModel` that is always in meters.
  * Input: User created `Targets` and `Waypoints` can have a `.native_scale` attribute to specify the scale.
    * Both classes have `.normalize_to_meters()` and `.normalized_to_meters()` functions to convert the target to meters.
  * Output: Forward kinematics functions can return frames at other scales by setting the optional `native_scale` parameter.
  * Output: `SceneObject` classes for visualization have a `native_scale` attribute to specify the drawing scale.
  * Input / Output: `Configuration` and `Trajectory` objects always uses the Meter-Radian units, in accordance with ROS convention.
    * Conversion of joint values to other units for execution should be done by the user based on robot controller requirements.
  * All the `native_scale` attribute has the same meaning where `user_object_value * native_scale = meter_object_value`.
    * In typical use, all `native_scale` attribute can be set to the same value, regardless of input or output.
    * For example, if the user wants to work with millimeters, set all `native_scale` to `'0.001'`.
* Added `compas_fab.robots.RobotCell` class.
* Added `compas_fab.robots.RobotCellLibrary` class.
* Added `compas_fab.backends.BackendTargetNotSupportedError`.
* Added `compas_fab.backends.TargetModeMismatchError`.
* Added `compas_fab.backends.PlanningGroupNotExistsError`.
* Added `compas_fab.backends.CollisionCheckError`.
* Added `compas_fab.backends.MotionPlanningError`.
* Added `compas_fab.backends.MPStartStateInCollisionError`.
* Added `compas_fab.backends.MPTargetInCollisionError`.
* Added `compas_fab.backends.MPInterpolationInCollisionError`.
* Added `compas_fab.backends.MPSearchTimeOutError`.
* Added `compas_fab.backends.MPNoIKSolutionError`.
* Added `compas_fab.backends.MPNoPlanFoundError`.
* Added `compas_fab.backends.MPMaxJumpError`.
* Added `compas_fab.backends.AnalyticalKinematicsClient`.
* Added `compas_fab.backends.AnalyticalKinematicsPlanner`.
* Added `compas_fab.backends.AnalyticalPyBulletPlanner`.
* Added `compas_fab.backends.CollisionCheckError`.
* Added `compas_fab.backends.PlanningGroupNotSupported`.
* Added `message` and `target_pcf` attributes to `InverseKinematicsError`.
* Added `compas_fab.backends.CheckCollision` to backend features.
* Added `compas_fab.backends.SetRobotCell` to backend features.
* Added `compas_fab.backends.SetRobotCellState` to backend features.
* Added `compas_fab.backends.ClientInterface.robot_cell` as read-only property.
* Added `compas_fab.backends.ClientInterface.robot_cell_state` as read-only property.
* Added `compas_fab.backends.ClientInterface.robot_model` as read-only property.
* Added `compas_fab.backends.ClientInterface.robot_semantics` as read-only property.
* Added `compas_fab.backends.PlannerInterface.set_robot_cell()` as a possible mix-in method.
* Added `compas_fab.backends.PlannerInterface.set_robot_cell_state()` as a possible mix-in method.
* Added `compas_fab.backends.PlannerInterface.check_collisions()` as a possible mix-in method.
* Added `compas_fab.backends.PlannerInterface.iter_inverse_kinematics()` as a possible mix-in method.
* Added `compas_fab.backends.kinematics.backend_features.AnalyticalPlanCartesianMotion` for `AnalyticalKinematicsPlanner`.
* Added `compas_fab.backends.kinematics.backend_features.AnalyticalForwardKinematics` for `AnalyticalKinematicsPlanner`.
* Added `compas_fab.backends.kinematics.backend_features.AnalyticalInverseKinematics` for `AnalyticalKinematicsPlanner`.
* Added `compas_fab.backends.kinematics.backend_features.AnalyticalSetRobotCell` for `AnalyticalKinematicsPlanner`.
* Added `compas_fab.backends.kinematics.backend_features.AnalyticalSetRobotCellState` for `AnalyticalKinematicsPlanner`.
* Added `compas_fab.backends.kinematics.backend_features.AnalyticalPybulletInverseKinematics` for `AnalyticalPyBulletPlanner`.
* Added `compas_fab.backends.pybullet.backend_features.PyBulletCheckCollision` for `PyBulletPlanner`.
* Added `compas_fab.backends.pybullet.backend_features.PyBulletPlanCartesianMotion` for `PyBulletPlanner`.
* Added `compas_fab.backends.pybullet.backend_features.PyBulletSetRobotCell` for `PyBulletPlanner`.
* Added `compas_fab.backends.pybullet.backend_features.PyBulletSetRobotCellState` for `PyBulletPlanner`.
* Added `compas_fab.backends.pybullet.backend_features.MoveItSetRobotCell` for `MoveItPlanner`.
* Added `compas_fab.backends.pybullet.backend_features.MoveItSetRobotCellState` for `MoveItPlanner`.
* Added `compas_fab.backends.ForwardKinematics.forward_kinematics_to_link()`.
* Added `compas_fab.robots.ToolLibrary` class.
* Added `compas_fab.robots.ToolLibrary.cone()`.
* Added `compas_fab.robots.ToolLibrary.printing_tool()`.
* Added `compas_fab.robots.ToolLibrary.static_gripper()`.
* Added `compas_fab.robots.ToolLibrary.static_gripper_small()`.
* Added `compas_fab.robots.ToolLibrary.kinematic_gripper()`. (example of a kinematic tool)
* Added `compas_fab.robots.RigidBodyLibrary` class.
* Added `compas_fab.robots.RigidBodyLibrary.target_marker()`.
* Added `compas_fab.robots.RobotCellLibrary` class.
* Added `compas_fab.robots.RobotCellLibrary.rfl()`.
* Added `compas_fab.robots.RobotCellLibrary.ur5()`.
* Added `compas_fab.robots.RobotCellLibrary.ur10e()`.
* Added `compas_fab.robots.RobotCellLibrary.abb_irb4600_40_255()`.
* Added `compas_fab.robots.RobotCellLibrary.abb_irb120_3_58()`.
* Added `compas_fab.robots.RobotCellLibrary.panda()`.
* Added `compas_fab.robots.RobotCellLibrary.ur5_cone_tool()`.
* Added `compas_fab.robots.RobotCellLibrary.abb_irb4600_40_255_gripper_one_beam()`.
* Added `compas_fab.robots.RobotCellLibrary.ur10e_gripper_one_beam()`.
* Added `compas_fab.robots.RobotCellLibrary.abb_irb4600_40_255_printing_tool()`.
* Added `compas_fab.robots.RigidBody` class.
* Added `compas_fab.robots.RigidBodyState` class.
* Added `compas_fab.robots.RobotCell` class.
* Added `compas_fab.robots.RobotCellState` class.
* Added `compas_fab.robots.ToolState` class.
* Added `compas_fab.robots.RobotCell.from_urdf_and_srdf()`
* Added `compas_fab.robots.Target` class.
* Added `compas_fab.robots.ConfigurationTarget` class.
* Added `compas_fab.robots.ConstraintSetTarget` class.
* Added `compas_fab.robots.FrameTarget` class.
* Added `compas_fab.robots.PointAxisTarget` class.
* Added `compas_fab.robots.TargetMode` class.
* Added `compas_fab.robots.Waypoints` class.
* Added `compas_fab.robots.FrameWaypoints` class.
* Added `compas_fab.robots.PointAxisWaypoints` class.
* Added `compas_fab.robots.FrameWaypoints` class.
* Added docker file for UR5-demo with GUI turned on by default.
* Added various example files (detailed list to be added later).
* Added `pragma: no cover` to all type-checking imports for better coverage report accuracy.
* Added support for python '3.11'.
* Added tests for `PyBulletClient` and `PyBulletPlanner` backend features, including Ik and FK agreement tests.

### Changed

* Changed CI workflow for IronPython.
* Updated compas requirement to > 2.3
* Changed `client` parameter in `PlannerInterface(client)` to default to `None`.
* Changed `PlannerInterface.client` to a read-only property.
* Moved `PlannerInterface` from `backends/interfaces/client.py` to `backends/interfaces/planner.py`
* Moved `AnalyticalInverseKinematics` to be a backend feature of `AnalyticalKinematicsPlanner`.
* Moved `AnalyticalPlanCartesianMotion` to be a backend feature of `AnalyticalKinematicsPlanner`.
* Change backend features to use multi-inherence instead of `__call__` to include the backend functions.
* Changed parameters of `compas_fab.backends.SetRobotCell.set_robot_cell()` to accept `RobotCell` and `RobotCellState`.
* Changed parameters of `compas_fab.backends.SetRobotCellState.set_robot_cell_state()` to accept `RobotCellState`.
* Changed parameters of `compas_fab.backends.ForwardKinematics.forward_kinematics()` to accept `RobotCellState`, `TargetMode` and `scale`.
* Changed parameters of `compas_fab.backends.InverseKinematics.inverse_kinematics()` to accept `Target` and `RobotCellState`.
* Changed parameters of `compas_fab.backends.InverseKinematics.iter_inverse_kinematics()` to accept `Target` and `RobotCellState`.
* Changed parameters of `compas_fab.backends.PlanMotion.plan_motion()` to accept `Target` and `RobotCellState`.
* Changed parameters of `compas_fab.backends.PlanCartesianMotion.plan_cartesian_motion()` to accept `Waypoints` and `RobotCellState`.
* Changed `compas_fab.robots.BoundingVolume` to inherit from `compas.data.Data`.
* Changed `compas_fab.robots.Constraint` to inherit from `compas.data.Data`.
* Changed `compas_fab.robots.JointConstraint` to inherit from `compas.data.Data`.
* Changed `compas_fab.robots.OrientationConstraint` to inherit from `compas.data.Data`.
* Changed `compas_fab.robots.PositionConstraint` to inherit from `compas.data.Data`.
* Moved `Robot.orientation_constraint_from_frame()` to `OrientationConstraint.from_frame()`.
* Moved `Robot.position_constraint_from_frame()` to `PositionConstraint.from_frame()`.
* Moved `Robot.constraints_from_frame()` to non-exposed `compas_fab.backends.ros.backend_features.convert_target_to_goal_constraints()`.
* Changed `avoid_collisions` parameter in `plan_motion` to `allow_collisions` parameter.
* Fixed error in `PyBulletForwardKinematics.forward_kinematics` where function would crash if `options` was not passed.
* Fixed error in `PyBulletInverseKinematics._accurate_inverse_kinematics` where threshold was not squared for comparison.
* Changed the behavior of `Duration` class when accepting both seconds (float) and nanoseconds (int) where the decimal point of seconds and the nanoseconds add up to more than 1 second.
* Changed GH Component `ConstraintsFromPlane` to `FrameTargetFromPlane`.
* Changed GH Component `ConstraintsFromTargetConfiguration` to `ConfigurationTarget`.
* Changed `RosClient` constructor to take a type for planner backend instead of a string. This also changes the name of the argument from `planner_backend` to `planner_type`.
* Changed type-hints from comment-style to standard Python 3.x type-hints
* Changed `RosDistro` to be an enum instead of string.
* Changed `TargetMode` to be an enum instead of string.
* Changed default values for number of planning attempts in MoveIt to 5, and the allowed planning time to 1 second.
* Changed cartesian motion planner of MoveIt to raise an exception (including the partial trajectory) if fraction is below 1.0 (ie. the trajectory is not possible).
* Changed `PyBulletClient.connect()` and `PyBulletClient.disconnect()` to take a `verbose` parameter.
* Changed `PyBulletClient._handle_concavity()` to ignore the `redirect_stdout` context manager to avoid the mysterious `WinError 6: The handle is invalid.` error.

### Removed

* Removed support for Python '3.8'.
* Removed `compas_fab.backends.CollisionError`.
* Removed `compas_fab.backends.AddCollisionMesh` from backend features.
* Removed `compas_fab.backends.AppendCollisionMesh` from backend features.
* Removed `compas_fab.backends.RemoveCollisionMesh` from backend features.
* Removed `compas_fab.backends.AddAttachedCollisionMesh` from backend features.
* Removed `compas_fab.backends.RemoveAttachedCollisionMesh` from backend features.
* Removed `compas_fab.backends.ClientInterface.planner` attribute.
* Removed `compas_fab.backends.ClientInterface.inverse_kinematics()`.
* Removed `compas_fab.backends.ClientInterface.forward_kinematics()`.
* Removed `compas_fab.backends.ClientInterface.plan_cartesian_motion()`.
* Removed `compas_fab.backends.ClientInterface.plan_motion()`.
* Removed `compas_fab.backends.ClientInterface.get_planning_scene()`.
* Removed `compas_fab.backends.ClientInterface.reset_planning_scene()`.
* Removed `compas_fab.backends.ClientInterface.add_collision_mesh()`.
* Removed `compas_fab.backends.ClientInterface.remove_collision_mesh()`.
* Removed `compas_fab.backends.ClientInterface.append_collision_mesh()`.
* Removed `compas_fab.backends.ClientInterface.add_attached_collision_mesh()`.
* Removed `compas_fab.backends.ClientInterface.remove_attached_collision_mesh()`.
* Removed `compas_fab.backends.pybullet.backend_features.PyBulletAddAttachedCollisionMesh`.
* Removed `compas_fab.backends.pybullet.backend_features.PyBulletAddCollisionMesh`.
* Removed `compas_fab.backends.pybullet.backend_features.PyBulletAppendCollisionMesh`.
* Removed `compas_fab.backends.pybullet.backend_features.PyBulletRemoveAttachedCollisionMesh`.
* Removed `compas_fab.backends.pybullet.backend_features.PyBulletRemoveCollisionMesh`.
* Removed `compas_fab.backends.pybullet.backend_features.MoveItAddAttachedCollisionMesh`.
* Removed `compas_fab.backends.pybullet.backend_features.MoveItAddCollisionMesh`.
* Removed `compas_fab.backends.pybullet.backend_features.MoveItAppendCollisionMesh`.
* Removed `compas_fab.backends.pybullet.backend_features.MoveItRemoveAttachedCollisionMesh`.
* Removed `compas_fab.backends.pybullet.backend_features.MoveItRemoveCollisionMesh`.
* Removed `compas_fab.backends.PyBulletClient.collision_objects` attribute.
* Removed `compas_fab.backends.PyBulletClient.attached_collision_objects` attribute.
* Removed `compas_fab.backends.PyBulletClient.load_ur5()`.
* Removed `compas_fab.backends.PyBulletClient.load_robot()`.
* Removed `compas_fab.robots.AttachedCollisionMesh`.
* Removed `compas_fab.robots.CollisionMesh`.
* Removed `compas_fab.robots.PlanningScene`.
* Removed `compas_fab.robots.Robot`. (Use `RobotCell` instead)
* Removed `compas_fab.robots.RobotLibrary`. (Use `RobotCellLibrary` instead)
* Removed `compas_fab.robots.Tool`. (Use `ToolModel` directly in `RobotCell` instead)
* Removed `compas_fab.robots.Robot.transformed_frames`. (use the one in `RobotModel` instead)
* Removed `compas_fab.robots.Robot.transformed_axes`. (use the one in `RobotModel` instead)
* Removed `compas_fab.robots.Robot.merge_group_with_full_configuration` as it can be covered by `Configuration.merged`.
* Removed `compas_fab.robots.Robot.get_position_by_joint_name` as Configuration class can now be directly accessed by joint name.
* Removed `compas_fab.robots.Robot.get_group_names_from_link_name` as it is too oddly specific.
* Removed `compas_fab.robots.JointTrajectory.attached_collision_meshes` attribute from `` class.
* Removed `inverse_kinematics`, `plan_cartesian_motion`, and `plan_motion` methods from Robot, access them using the planner instead.
* Removed `plan_cartesian_motion_deprecated` and `plan_motion_deprecated` methods from `Robot` class
* Removed `forward_kinematics_deprecated` and `inverse_kinematics_deprecated` method from `Robot` class
* Removed `compas_fab.sensors` module.
* Removed support for IronPython.

## [1.1.0] 2025-04-17

### Added

### Changed

* Made `pybullet` entirely optional. To install `pybullet`, use `pip install compas_fab.[pybullet]` or install `pybullet` manually.

### Removed


## [1.0.5] 2025-04-17

### Added

### Changed

* Rhino CPython support: change `pybullet` to be optional requirements inside Rhino.

### Removed


## [1.0.4] 2025-04-15

### Added

### Changed

### Removed


## [[1.0.3] 2025-04-15]

### Added

* Added helper function `message` to `compas_fab.ghpython.components`.
* Added helper function `error` to `compas_fab.ghpython.components`.
* Added helper function `remark` to `compas_fab.ghpython.components`.
* Added helper function `warning` to `compas_fab.ghpython.components`.
* Added GH component definitions compatible with CPython in Rhino8.

### Changed

* Updated dev dependency to `compas_invocations2`.
* Fixed `AttributeError` in `inverse_kinematics_spherical_wrist()`.
* Fixed `AttributeError` in VisualizeRobot GH component.

### Removed

* Removed `create_id` from `compas_fab.ghpython.components`, using `compas_ghpython.create_id` instead.


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
