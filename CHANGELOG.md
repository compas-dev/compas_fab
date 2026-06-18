# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Added

* New `MoveItCheckCollision` backend feature: `MoveItPlanner` now implements the `CheckCollision` interface (the same `planner.check_collision(state)` as `PyBulletPlanner`), backed by MoveIt's `/check_state_validity` service (`moveit_msgs/GetStateValidity`). It does a stateless collision + constraint check of the state against the planning scene currently loaded into `move_group` — no planning, no IK — and raises `CollisionCheckError` listing the colliding body pairs. Adds `GetStateValidityRequest` / `GetStateValidityResponse` ROS message wrappers.
* The `Inverse Kinematics`, `Plan Motion` and `Plan Cartesian Motion` Grasshopper components now explain a failure with a concise collision check on the start state, via a shared `collision_diagnostic` helper in `compas_fab.ghpython`: when the planner supports collision checking and the start state is in collision, the colliding pairs (e.g. `forearm_link<->shoulder_link, +2 more`) are appended; otherwise it notes the state is collision-free (likely a reachability issue). Planners without collision checking are handled gracefully. Each of these components also gained a `debug_info` output that mirrors the warning/error messages as a single string, so they can be read in a wired panel rather than only the component balloon.
* New `Deconstruct Robot Cell` Grasshopper component that splits a `RobotCell` into its `robot_model`, registered `rigid_bodies` and `tools`, and a matching `default_cell_state`, the inverse of registering models via the `Load Robot Cell` components.
* New `Deconstruct Planner` Grasshopper component that outputs the `robot_cell` and `robot_model` currently held by any planner (MoveIt, PyBullet, analytical). The cell state is intentionally not exposed — the planner's last-set state is rarely the one you want; build it explicitly via `Default Cell State` / the `Attach*` components instead.
* New `MoveIt Planner Options` Grasshopper component (and `MoveItPlannerOptions` class in `compas_fab.ghpython`) that bundles the advanced load parameters (`urdf_param_name`, `srdf_param_name`, `http_file_server_base_url`) into a single object for the `MoveIt Planner`'s optional `options` input. It is a plain class rather than a dict/namedtuple so it crosses a Grasshopper wire as one object instead of being exploded into its keys.
* `Load Robot Cell From Library` and `MoveIt Planner` auto-create a Boolean Toggle on their `load_geometry` input, and `ROS Client` auto-creates one (defaulting to off) on its `connect` input — via the new `ensure_boolean_toggle` helper in `compas_fab.ghpython`, so these can be flipped on/off with one click instead of wiring a separate toggle.
* New `RobotCell.structural_signature()` method: a stable fingerprint (robot model name + sorted tool and rigid-body ids) of a cell's structural identity. Consolidates a helper that was previously duplicated as a private function in `ActionChain`, and is reused by the `MoveIt Planner` component to decide when the planning scene needs re-uploading.
* `FrameInterpolator` and `PointAxisInterpolator` are now public (`compas_fab.robots`). They were private helpers buried in the PyBullet Cartesian-motion backend feature, but are backend-agnostic (only depend on `compas.geometry`) and useful on their own for interpolating between poses / point-axis pairs with bounded Cartesian and angular steps. Their constructors now take explicit keyword arguments (`max_step_distance`, `max_step_angle`, `min_step_distance`, `min_step_angle`) instead of an opaque `options` dict.
* New API documentation pages for the analytical-kinematics backend (`compas_fab.backends.kinematics` and `compas_fab.backends.kinematics.backend_features`), which had no API page while the ROS and PyBullet backends did — so `AnalyticalKinematics`, `AnalyticalKinematicsClient`, `AnalyticalForwardKinematics`, `AnalyticalPybulletInverseKinematics`, `AnalyticalSetRobotCell` and `AnalyticalSetRobotCellState` were undocumented. `AnalyticalKinematics` (the solver base class) is now also re-exported from `compas_fab.backends.kinematics`.
* `compas_fab.rhino` now re-exports its scene objects (`RigidBodyObject`, `RobotCellObject`, `RobotModelObject`) like `compas_fab.ghpython` already did, so they appear in the API docs instead of being hidden in a submodule.
* `MoveItResetPlanningScene` is now exported from `compas_fab.backends.ros.backend_features` (it was the only one of the eight MoveIt backend-feature classes missing from the package `__init__`/`__all__`, so it never showed up in the docs).

* New backend page [Analytical IK + PyBullet](backends/analytical_pybullet.md) for `AnalyticalPyBulletPlanner`, the hybrid planner that pairs closed-form analytical IK with PyBullet collision checking. Previously this backend was undocumented despite being shipped in `compas_fab.backends`.
* `docs/backends/pybullet.md` now flags that free-space `plan_motion` is not yet implemented for `PyBulletPlanner` (Cartesian motion via `plan_cartesian_motion` works).
* New `ros2-ur10e-demo` docker reference backend (ROS 2 Jazzy + MoveIt 2 + UR10e) with a `ur-sim` service running the official `universalrobots/ursim_e-series` Polyscope simulator, a `zenoh-router` service running `rmw_zenohd` as the RMW transport (replaces DDS, no more multicast discovery issues on Docker Desktop), a `ur-driver` service running the real `ur_robot_driver` against the simulator (with the ros2_control + RTDE update rate lowered from the default 500 Hz to 100 Hz via a baked-in `update_rate.yaml`, to stop the controller manager from spamming "Overrun detected!" warnings under Docker virtualisation), `moveit-demo` running `ur_moveit.launch.py`, rosbridge on `9090`, an HTTP file server on `9091` for serving meshes from `/opt/ros/jazzy/share/`, and a `theasp/novnc` web GUI on `8080` for viewing RViz.
* New `compas_fab.backends.HttpFileServerLoader` that mirrors `RosFileServerLoader`'s interface but fetches meshes over plain HTTP and reads URDF/SRDF from rosbridge topics (the ROS 2 convention) instead of ROS parameters.
* Migrated documentation from Sphinx to MkDocs Material, matching the structure used in `compas_robots`. New `mkdocs.yml` at the repository root; documentation sources are now Markdown (`docs/*.md`) with API pages driven by `mkdocstrings`. Backlinks are disabled. `inventories` includes `compas`, `compas_robots`, and `compas_viewer`. Sphinx config (`docs/conf.py`) and Sphinx-only `docs/requirements.txt` have been removed; `tasks.py` now invokes `compas_invocations2.mkdocs.docs` so `invoke docs` builds the MkDocs site.
* New developer guide section: [Backend architecture](developer/architecture.md) ported from the old `docs/developer/backends.rst`, plus [Grasshopper components](developer/grasshopper.md) ported from `docs/developer/grasshopper.rst`.
* New [Icon system](developer/icons/index.md) page documenting the Grasshopper component icon design system (24×24 artboard, 1.8 px monoline keyline, shared primitive kit — cube=cell, square=body, cone=tool, triad=frame, ring=goal, polyline=motion, brackets=library, chip=backend, teal accent for the semantic subject). Ships the editable SVG source (`icons.js`) and a standalone catalog/spec sheet (`icon_system.html`) showing all glyphs grouped by subcategory with a Grasshopper-style preview.
* Re-rendered every `Cf_*` component `icon.png` from the new design system — consistent stroke weight, shared primitive kit, teal accent for the semantic subject of each icon. Affects every `src/compas_fab/ghpython/components_cpython/*/icon.png`.
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
* Added `compas_fab.robots.RobotCell.from_urdf_and_srdf()`. Absolute paths passed for `local_package_mesh_folder` are now used as-is; relative paths still resolve against `compas_fab.DATA` (the existing shorthand for the bundled robot library, e.g. `"robot_library/ur5_robot"`, keeps working). Previously, routing absolute paths through `compas_fab.get()` stripped their leading slash and produced a malformed `<DATA>/<absolute-path>` join that never resolved.
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
* Redesigned the Grasshopper component library (CPython / Rhino 8) for the new stateless planning model. The legacy IronPython `components/` set was retired; all components now live under `src/compas_fab/ghpython/components_cpython/` and target Rhino 8 CPython exclusively. New categories under "COMPAS FAB":
  * *Backends* — `Cf_RosClient` (with a `transport` input — `twisted`/`asyncio`/`cli` — forwarded to `roslibpy.Ros` via `RosClient`'s new `**kwargs` pass-through; falls back to roslibpy's process-wide default when empty; the input requires roslibpy >= 2.1 and is ignored with a warning on older versions), `Cf_MoveItPlanner`, `Cf_AnalyticalKinematicsPlanner`, `Cf_PyBulletPlanner` (in-process collision checking, FK/IK and Cartesian motion) and `Cf_AnalyticalPyBulletPlanner` (closed-form analytical IK paired with PyBullet collision checking, via a solver dropdown). Both PyBullet planner components manage their own in-process PyBullet client — pass a `RobotCell` and optionally pick the connection type (`direct`/`gui` via an auto-created dropdown); the component connects, builds the planner, applies `set_robot_cell`, and caches the client across canvas refreshes, so there is no separate client component to wire.
  * *Robot Cell* — `Cf_LoadRobotCellFromLibrary` (with an auto-created dropdown listing every `RobotCellLibrary` entry), `Cf_LoadRobotCellFromUrdfSrdf`, `Cf_RigidBodyFromMesh`, `Cf_ToolFromLibrary`, `Cf_RigidBodyFromLibrary`. The two `LoadRobotCell*` components each take `tools` and `rigid_bodies` list inputs and register them into the cell as part of loading (keyed by each item's `.name`), so a complete cell reaches the planner in one shot — there is no separate `Add* → set_robot_cell()` step to forget. (Loading from ROS works the same way but is folded into the `MoveIt Planner` component, since MoveIt's cell can only come from `move_group`.) `RigidBody` gained a `name` field (surfaced as the `name` / `rigid_body_id` inputs on the rigid-body builders) so a body is self-describing like a `ToolModel`.
  * *Cell State* — `Cf_DefaultCellState`, `Cf_SetRobotConfiguration`, `Cf_AttachToolToRobot` (auto-creates a dropdown of every tool in the wired cell when `tool_id` is unwired, refreshed when the cell's tool set changes; emits a remark naming the actual link the tool ends up attached to, e.g. `tool0` for UR groups — `attachment_plane` corrects any per-robot EE-axis convention quirks) and `Cf_AttachRigidBodyToLink` (both auto-default `touch_links` from the cell when unwired — `AttachToolToRobot` picks the group's end-effector link plus its parent when the EE link is geometry-less, e.g. UR `tool0` → `flange`; `AttachRigidBodyToLink` picks the link the body is attached to. A warning surfaces the auto-pick so it isn't silent), `Cf_AttachRigidBodyToTool`, `Cf_SetRigidBodyFrame`, `Cf_SetTouchLinks`.
  * *Targets* — `Cf_FrameTarget`, `Cf_PointAxisTarget`, `Cf_ConfigurationTarget`, `Cf_FrameWaypoints`, `Cf_PointAxisWaypoints` (tightened `points` input to `list[Rhino.Geometry.Point3d]` so the script harness marshals each item as a real `Point3d` rather than the per-coord float flattening produced by the generic `List[object]` typing), `Cf_RobotConfiguration` (auto-creates one `GH_NumberSlider` per configurable joint with the joint's limits, all wired to a single `joints` list input — drop and wire a `robot_cell` and the slider bank appears; values flow through the standard GH solve path).
  * *Planning* — `Cf_ForwardKinematics`, `Cf_InverseKinematics` (accepts a bare `compas.geometry.Frame` or Rhino `Plane` as `target` and auto-wraps it as `FrameTarget(target_mode=ROBOT)`; defaults `start_state` to a zero-config state derived from `planner.robot_cell` when unwired; warns when `planner` or `target` are wired but received None; emits both the `configuration` *and* an updated `cell_state` so `Visualize` / `ForwardKinematics` / the next planner step can wire straight in without a `Cf_SetRobotConfiguration` step in between), `Cf_PlanMotion` and `Cf_PlanCartesianMotion` (with `planes` and `polyline` outputs — Rhino planes at the planning group's EE per trajectory point and a polyline through their origins — via `JointTrajectory.to_frames_and_polyline`, computed locally through `RobotCell.forward_kinematics_target_frame` so no round trip per point. Planning failures flag the component red with the backend error message; no separate `error` output), `Cf_DeconstructTrajectory` (expands a `JointTrajectory` into a list of `Configurations`, parallel `RobotCellStates` derived from `trajectory.start_state`, and DataTrees of `velocities`/`accelerations`/`efforts` — one branch per point — for plotting the trajectory profile; pipe an index slider into the `cell_states` output to scrub through `VisualizeRobotCell`).
  * *Display* — `Cf_VisualizeRobotCell` (uses the `RobotCellObject` scene object with native-geometry caching for smooth slider scrubbing).
  * *ROS* — `Cf_RosTopicPublish`, `Cf_RosTopicSubscribe` (ported to the new `RosClient`).
  * Planner calls take a `RobotCell` + `RobotCellState` + `Target` / `Waypoints` triple. Long-lived clients/planners use `scriptcontext.sticky`; user messages flow through `compas_ghpython.error`/`warning`/`remark`. Loader errors are caught and surfaced via clean error messages rather than letting tracebacks reach the GH canvas.
* Added `Cf_TrajectoryAction`, `Cf_StateChangeAction`, and `Cf_ActionChain` Grasshopper components under *Planning*. The two action builders wrap a `JointTrajectory` (or a `RobotCellState` for a discrete state change) into a named `Action` next to the data that produced it — names cannot drift out of sync with the underlying object — and accept an optional comma-separated `tags` input that lands on the action's `attributes`. `Cf_ActionChain` takes the ordered `actions` list, an optional `robot_cell` (for signature verification on load), and emits the assembled `chain` plus composite visualisation outputs: a DataTree of EE planes with one branch per trajectory action, a parallel list of polylines (one per action), `duration` and `end_state`. The viz reuses `trajectory.to_frames_and_polyline` per action, walking the chain's state sequence so each segment's FK uses the correct pre-state. The helper's signature was relaxed to take `robot_cell` directly (instead of digging through `planner.client.robot_cell`), making it reusable from any caller that already has a cell.
* Added `compas_fab.robots.Action` and `compas_fab.robots.ActionChain` — a higher-level assembly layer that threads a single `RobotCellState` through a sequence of `Action` objects. A single `Action` class covers both kinds: one carrying a `JointTrajectory` is a *planned* action and derives its post-state from the predecessor (last-point applied); one without a trajectory is an *unplanned* (state-change) action (gripper toggle, tool attach/detach) and carries an explicit post-state. Using one class means backtracking/undo just clears the trajectory instead of swapping class types. Each action also carries `tags` (over a free-form `attributes` bag) to drive differentiated downstream execution (e.g. `approach`/`retract`, linear vs. free motion). The canonical use case is pick-and-place: `ActionChain(name, start_state).append_trajectory(...).append_state_change(...)...`, fluent chaining. Lookup is by name (`chain.action_by_name(...)`) rather than index, so a future tree/branching extension won't force an API break. Optional `robot_cell` constructor argument stores a structural signature (robot name + tool/body ids, SHA-256) so `chain.verify_cell(cell)` fails loudly when a chain is loaded against the wrong cell. The chain owns the state sequence: each `Action.start_state` is mirrored onto the contained trajectory's `start_state`, and serialisation strips both (re-threaded on load) and omits the derived `post_state` of planned actions; only the chain's `start_state` and explicit state-change post-states are stored. Planners' return types are unchanged — `ActionChain` is purely an additive composition layer.
* Added `compas_fab.robots.JointTrajectory.start_state` — an optional `RobotCellState` that carries the full cell context the trajectory was planned from (tools, rigid bodies, attached collision objects, robot configuration). MoveIt, PyBullet and analytical Cartesian planners now populate it. **Breaking change**: the `JointTrajectory` constructor signature is now `(trajectory_points, joint_names, start_state, fraction, attributes)` — `start_configuration` has been removed from the constructor and is set via the property only (`traj.start_configuration = config` after construction). The getter returns the explicit override if set, otherwise `start_state.robot_configuration`. `Cf_DeconstructTrajectory` derives its per-point cell states straight from `trajectory.start_state` (the `start_state` input was dropped — the trajectory carries it), and the `trajectory_to_planes_and_polyline` viz helper falls back to `trajectory.start_state` when its explicit `start_state` is unwired, so a planned trajectory now visualises end-to-end without re-wiring the cell state on the output side.
* Added `compas_fab.ghpython.ensure_value_list`, a reusable helper for components that want to auto-create a Grasshopper `Value List` on an unconnected input (used by `Cf_LoadRobotCellFromLibrary` and intended for follow-on components like `Cf_FrameTarget`'s `target_mode` and solver-name pickers).
* Added `JointTrajectory.to_frames_and_polyline` to ease visualization of trajectories.

### Changed

* `JointTrajectory.to_frames_and_polyline` gained an optional `target_mode` parameter (defaults to `TargetMode.ROBOT`), so the visualized path can be reported at the robot's planner frame, the attached tool's TCF, or the workpiece. The `Plan Motion` and `Plan Cartesian Motion` Grasshopper components now pass the `target_mode` of their `target` / `waypoints` through to it, so the previewed `planes` / `polyline` match what was actually planned (e.g. the tool tip path for a `TOOL`-mode target).
* The `MoveIt Planner` Grasshopper component now loads the robot cell from ROS itself, and the separate `Load Robot Cell From ROS` component has been removed. MoveIt's planning scene is defined by the URDF/SRDF loaded into `move_group`, so the cell can only come from ROS — merging the two removes a fragile two-step wiring (and the previous `ros_client` pass-through workaround that enforced load-before-plan ordering). `MoveIt Planner` now takes `ros_client`, `tools` / `rigid_bodies` (registered into the cell and uploaded as collision objects), `load_geometry`, `reload`, and an optional `options` input, and outputs `planner` (first), `robot_cell`, and `detected_distro`. The cell is fetched once and cached (set `reload` to refetch), and re-uploaded to the planning scene only when it actually changes. The advanced load parameters (`urdf_param_name`, `srdf_param_name`, `http_file_server_base_url`) moved to a new `MoveIt Planner Options` component, so the default case is just `RosClient → MoveIt Planner`. Use `Deconstruct Planner` to recover the loaded cell / model when needed.
* Reorganised the API reference into three groups that match the public-API surface: **Core** (`compas_fab.robots`, `compas_fab.backends`, `compas_fab.utilities`, `compas_fab.scene`) and **Integrations** (`compas_fab.blender`, `compas_fab.ghpython`, `compas_fab.rhino`) hold the classes end users call, while **Extending compas_fab** gathers the contributor material — the [Backend architecture](developer/architecture.md) guide, the backend interfaces and per-backend `*.backend_features` API pages, and the Grasshopper-component / icon-system / ActionChain notes (the former "Developer guide" section). With `inherited_members` enabled, each planner on the `compas_fab.backends` page now documents its full FK/IK/motion-planning method set directly (the methods it composes through multiple inheritance), so the feature mixins no longer need to be front-and-centre. The redundant per-backend pages (`compas_fab.backends.ros`/`.pybullet`/`.kinematics`) were removed — they were ~95–100% duplicates of the `compas_fab.backends` aggregator — and `AnalyticalKinematics` and `AnalyticalKinematicsClient` are now re-exported from `compas_fab.backends` so they remain documented in Core.
* Unified the collision-checking option key across backends to `check_collision` (default `True` = collision-free solutions only). Previously `MoveItInverseKinematics` used `allow_collision` (inverted semantics) while the PyBullet/analytical backends already used `check_collision`. The MoveIt feature now accepts the legacy `allow_collision` key as a backward-compatible alias — when `check_collision` is unset and `allow_collision` is present, the latter is inverted and used — so existing callers and the `docs/backends/ros/files/04_ik_allow_collision.py` example continue to work unchanged. The new `Cf_InverseKinematics` `check_collision` boolean input passes straight through with no UI-layer translation.
* Changed the default HTTP file server port used by `RosClient.load_robot_cell` (and `HttpFileServerLoader`) from `9091` to `9190`. The previous default collided with the rosbridge port `9091` commonly used when remapping a ROS 2 stack to coexist with a ROS 1 stack (rosbridge `9090` + ROS 2 rosbridge `9091`), causing HTTP 404s when the loader hit the bridge instead of the file server. `9190` stays clear of the 909x cluster while remaining mnemonic. The `docker-compose-ros2.yml` test stack and the `ros2-ur10e-demo` reference stack both default `ROS2_HTTP_PORT` and the container-internal port to `9190` to match.
* Standardized every backend page (`docs/backends/{analytical,analytical_pybullet,pybullet,ros,ros2}.md`) to a common structure: *When to use → Trade-offs → Setup → First example (embedded) → More examples (linked) → API reference*. Each page now embeds one runnable example from `docs/backends/*/files/` via `pymdownx.snippets` and links the remaining ~50 examples to GitHub.
* Rewrote `docs/backends/index.md` as a real decision guide: a "by intent" table keyed by what the user wants to do (rather than backend name), plus a "by capability" matrix and explicit setup-cost summary. Linked from both Home and Installation.
* Split `docs/installation.md` into a focused library-install page (uv/pip/conda + verify) and a new `docs/frontends.md` covering CAD environment setup (Rhino 8, Grasshopper, Blender, COMPAS Viewer, headless). The new front-ends page also documents which CAD environments work with which backends, surfacing the limitation that PyBullet cannot run inside Rhino's interpreter.
* Rebranded the old `tutorial.md` as `concepts.md`: a backend-agnostic walkthrough of the data model (`RobotCell`, `RobotCellState`, `Target`/`Waypoints`, `TargetMode`). No planner calls, the concepts page now reads as "this is the data; backends are how you execute it" and links out to the backends section for the actual planning calls.
* Rewrote `docs/index.md` as a real landing page (was a 1-paragraph blurb): leads with the five-backend table, "I want to..." quick links, and a four-step what's-next list. Updated MkDocs nav to surface the new structure (Home → Installation → CAD front-ends → Concepts → Backends → API → Developer guide).
* Bumped `compas_robots` requirement from `>=0.6,<1` to `>=1,<2`.
* Migrated packaging to `pyproject.toml`.
* Cleaned up `requirements-dev.txt`.
* Python classifiers trimmed to 3.9 (minimum, required for Rhino 8) and 3.13 (latest).
* Unpinned `roslibpy` from `1.8.1` to `>=2.0,<3` in preparation for ROS 2 support.
* Updated `ActionClient` and `Goal` imports from `roslibpy.actionlib` to `roslibpy.ros1.actionlib` following the breaking namespace change in `roslibpy` 2.x.
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

### Fixed

* `Plan Motion` and `Plan Cartesian Motion` Grasshopper components raised a `NullReferenceException` when planning failed with no trajectory: the visualization helper (and, for Cartesian, the `fraction` read) dereferenced a `None` trajectory instead of just flagging the component. Both now short-circuit on a `None` trajectory and output empty geometry.
* The PyBullet backend raised `NameError: name 'pybullet' is not defined` whenever the real `pybullet` module had already been imported elsewhere in the process (e.g. the user does `import pybullet` in the same Grasshopper script). The lazy-loader guard in `compas_fab.backends.pybullet.client` only bound the `pybullet` name when the module was *not* yet imported — being already in `sys.modules` never created the local binding. It now binds the (cached) real module in that case.
* Auto-created Grasshopper dropdowns and boolean toggles (`ensure_value_list` / `ensure_boolean_toggle`) duplicated themselves on every recompute, stacking 2–4 value lists on each `target_mode` input and extra toggles on `load_geometry`. The widget was added with `AddObject`/`AddSource` inline during the running solution, which doesn't commit until the solution ends, so follow-up solves still read `SourceCount == 0` and added another. Creation now happens inside the `ScheduleSolution` callback (between solves, where the wire commits), guarded by a sticky pending flag, so exactly one widget is ever added.
* The `Visualize Robot Cell` Grasshopper component failed with a `None` error when no `cell_state` was wired: it reads the `RobotCellObject`'s child scene objects directly, but those are only built when a state is applied via `update()`, which was skipped for an unwired state. It now falls back to the cell's `default_cell_state()`, drawing the cell in its default configuration.
* `RigidBodyLibrary.floor()` built a flat plane at `Z=0`, which collided with the robot base under distance-0 collision checking and made every configuration fail (so e.g. PyBullet IK with a floor returned no solutions). A robot base's collision geometry can dip several millimetres below the origin (the UR5 `base_link_inertia` reaches about -7 mm), so `floor()` now takes a `clearance` parameter and sits below `Z=0` by default (1 cm). Adding the base link to the floor's `RigidBodyState.touch_links` remains the robot-agnostic alternative.
* Outgoing ROS message headers were not JSON-serializable under `roslibpy` 2.0, breaking every `MoveItPlanner.set_robot_cell` (and any other header-carrying message) with `TypeError: Object of type Time is not JSON serializable`. `roslibpy` 2.0 wraps a header's `stamp` in a `UserDict`-based `roslibpy.core.Time`, which `json.dumps` rejects; `compas_fab`'s `std_msgs.Header.msg` / `format_header_for_distro` build outgoing headers via `dict(roslibpy.Header(...))`, embedding that `Time`. Both now flatten the roslibpy header (and its nested `stamp`) to plain `dict`s via a new `_to_plain` helper, keeping roslibpy's per-distro field shaping while emitting only JSON-native types. Regression-tested in `tests/backends/ros/messages/test_std_msgs.py::test_header_msg_is_json_serializable`.
* `PyBulletClient` could not be used outside a `with` block: `_cache_dir` was only created in `__enter__`, so calling `connect()` directly left it `None` and every mesh/tool/URDF load failed with `AttributeError: 'NoneType' object has no attribute 'name'`. `_cache_dir` is now a lazily-created property (the `TemporaryDirectory` is made on first access and still cleaned up on `__exit__`), so the client works whether or not it is used as a context manager — which is how the Grasshopper `PyBullet Client` component drives it.
* `redirect_stdout` (used throughout the PyBullet backend to silence native output) called `sys.stdout.fileno()` unconditionally, raising `io.UnsupportedOperation: fileno` inside embedded interpreters whose stdout has no OS-level file descriptor — e.g. Rhino 8's CPython, breaking `PyBulletClient.connect()` there. It already special-cased pytest and ipykernel for the same reason; it now also skips the fd-level redirect whenever `fileno()` is unavailable, so the PyBullet backend (and its new Grasshopper components) run in Rhino.
* `AnalyticalPybulletInverseKinematics._iter_inverse_kinematics_frame_target` was silently dropping every collision-free IK candidate. The `try`/`except CollisionCheckError` block had no `else` branch — when the collision check passed (no exception), the configuration was never yielded. Added the missing `else: yield configuration`. Verified end-to-end with the `03_iter_ik_pybullet.py` example, which now returns 6 valid + 2 colliding configurations (was: 0 valid + all 8 reported as colliding).
* `AnalyticalInverseKinematics.iter_inverse_kinematics` and its `_iter_inverse_kinematics_frame_target` helper had `group: Optional[str]` declared without a default, while the parent `InverseKinematics` interface and all example usages treat `group` as optional. Added `= None` so calls like `planner.iter_inverse_kinematics(target, start_state)` work as documented.
* Cleared stale docstring parameters across `compas_fab.backends` interfaces and backend feature implementations (`robot`, `client`, `solver`, `planner_type`) that no longer matched their signatures, plus a confusingly-indented continuation line in `RigidBodyState.attached_to_link`. `invoke docs` now builds with zero warnings.
* Fixed three analytical/PyBullet examples that crashed at startup due to cell-state desync after objects were added to the cell:
  * `docs/backends/analytical_kinematics/files/02_inverse_kinematics with_tools.py`: now refreshes the state from the cell after adding the cone tool.
  * `docs/backends/analytical_kinematics/files/03_analytical_pybullet_planner.py`: rewritten to use the pre-configured `RobotCellLibrary.abb_irb4600_40_255_printing_tool()` cell (which has the correct `touch_links` semantics) and derives its target frame via FK from a chosen seed pose. Was previously crashing on `KeyError: 'cone'` and then yielding 0 IK solutions even after that.
  * `docs/backends/pybullet/files/04_ik_semi_constrained.py`: now creates the `target_marker` rigid body state explicitly (was crashing on `KeyError: 'target_marker'`).
* `docs/backends/analytical_kinematics/files/03_iter_ik_pybullet.py` (already covered above) now derives its target via FK for clarity.
* `docs/backends/analytical_kinematics/files/04_cartesian_path_analytic_pybullet.py` now has a comment noting that `matplotlib` is an optional dependency that must be installed separately.
* All analytical-backend examples now import solver classes (`UR5Kinematics`, `ABB_IRB4600_40_255Kinematics`) from `compas_fab.backends` (the public top-level path), not the private `compas_fab.backends.kinematics.solvers`.
* `MoveItPlanMotion.plan_motion` and `MoveItPlanCartesianMotion.plan_cartesian_motion` were re-raising every typed planner error (`MPNoPlanFoundError`, `MPStartStateInCollisionError`, …) wrapped in `RosValidationError`, so callers catching `MotionPlanningError` as documented were missing every real failure. Both methods now unwrap `RosValidationError` and re-raise `e.original_exception`, matching `MoveItInverseKinematics.iter_inverse_kinematics`.
* `JointTrajectoryPoint.joint_names` was empty on points produced by the MoveIt and analytical Cartesian planners — the ROS message spec only carries names on the parent `JointTrajectory`, and the analytical path simply forgot. Both backends now attach the trajectory's `joint_names` per point at construction, matching PyBullet. Any consumer that walks points by name (the GH viz helper, `Cf_DeconstructTrajectory`) was silently FK-ing the start state for every point.
* `ToolLibrary.cone()` was not initializing a link on the underlying `ToolModel` when `load_geometry=False`, which left the tool's robot tree malformed and made any downstream `iter_joints()` / `get_configurable_joints()` call raise `AttributeError: 'NoneType' object has no attribute 'joints'`. As a result `RobotCellLibrary.ur5_cone_tool(load_geometry=False)` always crashed. The factory now calls `add_link("cone_link", visual_meshes=[...], collision_meshes=[...])` (mirroring `ToolLibrary.printing_tool()`) so the model is structurally valid in both geometry modes.
* Eliminated intermittent CI failures in `integration.yml` on the `RosClient` doctest under `pytest --doctest-modules` that raised `RosTimeoutError('Failed to connect to ROS')`. Both `>>> with RosClient() as client:` examples in `compas_fab.backends.ros.client` (the `RosClient` class docstring at line 162 and `RosClient.load_robot_cell` at line 257) are now marked `# doctest: +SKIP` — they remain as readable illustrations of the API but no longer run as live integration tests. The actual code path is exercised by `tests/backends/ros/test_ros_client.py` (9 cases against a module-scoped `ros1_client` fixture, never observed to flake). Earlier hypotheses — cold rosbridge startup (reverted in `2ffc37fac`) and a roslibpy 2.x reactor lifecycle issue (validated against [gramaziokohler/roslibpy@lifecycle-fixes](https://github.com/gramaziokohler/roslibpy/tree/lifecycle-fixes) via a temporary `requirements.txt` pin) — both failed to eliminate the flake, falsifying them; the root cause appears to be an interaction between doctest's evaluation context and roslibpy's twisted reactor that only the first `>>> with RosClient()` after the test-fixture suite triggers (the second such doctest always succeeds).

### Removed

* All Sphinx-era documentation sources: every `docs/**/*.rst`, the `docs/conf.py` Sphinx config, the `docs/_static/` and `docs/_images/` asset folders, the `docs/developer/generated/` autosummary output, `docs/spelling_wordlist.txt` and `docs/doc_versions.txt` (the latter superseded by `mike`'s gh-pages versioning).
* `.. autosummary::`, `.. toctree::` and `.. currentmodule::` directives from all module-level `__init__.py` docstrings (replaced with prose + intra-doc Markdown links resolved by `mkdocstrings`).
* Removed `DirectUrActionClient` and associated `direct_ur` messages module since this UR-specific action client was outdated and unused.
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


## [1.0.3] 2025-04-15

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
