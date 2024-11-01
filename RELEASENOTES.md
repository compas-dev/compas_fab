# Release Notes

## Unreleased - Implemented Features

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

## Unreleased - To be Implemented Features

* New `PlanMotion` function for `PyBulletPlanner`. Compatible with `FrameWaypoints` and `PointAxisWaypoints`.
* New Grasshopper Components
* `SceneObject` classes for Blender and Viewer
* Restructure tutorial and new example files
* Check and fix `ReachabilityMap`
* Check and fix `AnalyticalKinematics` and `AnalyticalPlanner`