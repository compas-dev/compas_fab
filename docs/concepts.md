# Tutorial

`compas_fab` connects robot models with motion planners and execution back-ends.
This page introduces the core building blocks. For installation, see
[Installation](installation.md). For deep dives on specific back-ends, see
[Backends](backends/index.md).

## Loading a robot cell

The [RobotCellLibrary][compas_fab.robots.RobotCellLibrary] provides ready-to-use
robot cells for prototyping. Each entry returns a
[RobotCell][compas_fab.robots.RobotCell] together with a default
[RobotCellState][compas_fab.robots.RobotCellState]:

```pycon
>>> from compas_fab.robots import RobotCellLibrary
>>> cell, state = RobotCellLibrary.ur10e()
>>> print(cell.robot_model)
Robot name=ur10e, Links=11, Joints=10 (6 configurable)
```

A [RobotCell][compas_fab.robots.RobotCell] is the workspace: it bundles a robot
model and semantics, any tools attached to its flanges, and any rigid bodies
(workpieces, fixtures) that participate in motion planning or collision checking.

A [RobotCellState][compas_fab.robots.RobotCellState] is a snapshot of the cell
at a moment in time: the robot's [Configuration][compas_robots.Configuration],
which tools and bodies are attached where, and their poses. State is what you
plan from and to.

## Targets and waypoints

A [Target][compas_fab.robots.Target] specifies where the robot should go.
Common targets:

- [FrameTarget][compas_fab.robots.FrameTarget] — a Cartesian pose for the tool
- [PointAxisTarget][compas_fab.robots.PointAxisTarget] — a point plus an axis
  (free rotation around the axis)
- [ConfigurationTarget][compas_fab.robots.ConfigurationTarget] — joint values

A [Waypoints][compas_fab.robots.Waypoints] object specifies a sequence of poses
to follow for Cartesian motion.

## Planning a motion

Planning happens through a back-end client and planner. The simplest is the
analytical inverse kinematics backend:

```pycon
>>> from compas_fab.backends import AnalyticalKinematicsPlanner
>>> from compas_fab.backends import UR10eKinematics
>>> planner = AnalyticalKinematicsPlanner(UR10eKinematics())
```

For full motion planning (collision checking, trajectory smoothing), use
ROS/MoveIt or PyBullet — see [Backends](backends/index.md).

## Visualization

`compas_fab` shares scene objects with [compas_robots][], so visualization
works in any supported front-end (Rhino, Grasshopper, Blender, COMPAS viewer)
without changing the planning code. See [Installation](installation.md) for
front-end setup.

## Next steps

- [Backends](backends/index.md) — connect to a planner
- [API Reference](api/index.md) — full module documentation
