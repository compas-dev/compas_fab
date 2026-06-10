# Choosing a backend

**COMPAS FAB** is one library that drives five different planning back-ends.
Pick the one that matches what you want to do.

## By intent

| I want to… | Use | Why |
|---|---|---|
| Just compute IK on a robot (e.g., UR, Staubli, ABB, etc) | [Analytical IK](analytical.md) | Closed-form, microsecond IK, no Docker, no PyBullet |
| Compute IK *and* check collisions | [Analytical IK + PyBullet](analytical_pybullet.md) | Analytical-fast IK filtered against a real collision scene |
| Plan motion (collisions + trajectory smoothing) without ROS | [PyBullet](pybullet.md) | In-process motion planning; runs anywhere Python runs |
| Plan motion via ROS 2 + MoveIt 2 | [ROS 2 + MoveIt 2](ros2.md) | Current ROS LTS; the recommended starting point for new ROS work |
| Drive an existing ROS 1 + MoveIt 1 setup | [ROS 1 + MoveIt 1](ros.md) | Legacy stack; only use if you must |
| Just model / visualize a robot cell in a CAD environment | *no backend* | The core data model works without any planner. See [Concepts](../concepts.md) |

## By capability

| Capability | Analytical | Analytical + PyBullet | PyBullet | ROS 1 + MoveIt 1 | ROS 2 + MoveIt 2 |
|---|:-:|:-:|:-:|:-:|:-:|
| Forward kinematics | ✓ | ✓ | ✓ | ✓ | ✓ |
| Inverse kinematics | ✓ (closed-form) | ✓ (closed-form) | ✓ (numerical) | ✓ | ✓ |
| Collision checking | — | ✓ | ✓ | ✓ | ✓ |
| Point-to-point motion planning | — | — | ✓ | ✓ | ✓ |
| Cartesian motion planning | — | ✓ (partial) | ✓ | ✓ | ✓ |
| Visualisation | — | PyBullet GUI | PyBullet GUI | RViz | RViz |
| Setup cost | none | `pip install pybullet` | `pip install pybullet` | Docker | Docker |
| Usable from inside Rhino 7/8 | ✓ | — | — | ✓ (over WebSocket) | ✓ (over WebSocket) |

## Setup cost in plain words

- **Analytical IK**: nothing to install beyond `compas_fab` itself.
- **PyBullet / Analytical + PyBullet**: one `pip install pybullet` (with a
  small workaround on macOS, see the per-backend pages).
- **ROS 1 & ROS 2**: Docker Desktop + the per-robot compose stack in
  [`docs/installation/docker_files/`](https://github.com/compas-dev/compas_fab/tree/main/docs/installation/docker_files).

## Without a backend

Even without a planning back-end, the core of `compas_fab` is useful for:

- Building [`RobotCell`][compas_fab.robots.RobotCell] instances with tools
  and rigid bodies
- Modelling cell state via [`RobotCellState`][compas_fab.robots.RobotCellState]
- Authoring [`Target`s][compas_fab.robots.Target] and
  [`Waypoints`][compas_fab.robots.Waypoints] and serializing them for
  planning elsewhere
- Visualising in a CAD front-end
- Forward kinematics via
  [`compas_robots.RobotModel.forward_kinematics`][]

For a backend-agnostic walkthrough of the data model, see
[Concepts](../concepts.md).
