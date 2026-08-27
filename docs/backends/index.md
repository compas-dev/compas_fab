# Choosing a backend

**COMPAS FAB** is one library that drives six different planning back-ends.
Pick the one that matches what you want to do.

## By intent

| I want to… | Use | Why |
|---|---|---|
| Just compute IK on a robot (e.g., UR, Staubli, ABB, etc) | [Analytical IK](analytical.md) | Closed-form, microsecond IK, no Docker, no PyBullet |
| Continuously servo a target with numerical IK | [PyRoKI](pyroki.md) | Differentiable IK with inexpensive warm solves and optional task-axis freedom |
| Compute IK *and* check collisions | [Analytical IK + PyBullet](analytical_pybullet.md) | Analytical-fast IK filtered against a real collision scene |
| Compute collision-aware IK for a redundant or custom robot | [PyRoKI](pyroki.md) | Joint-limit and differentiable approximate-collision constraints in one solve |
| Plan Cartesian motion without ROS | [PyBullet](pybullet.md) | In-process waypoint interpolation and collision checking |
| Plan motion via ROS 2 + MoveIt 2 | [ROS 2 + MoveIt 2](ros2.md) | Current ROS LTS; the recommended starting point for new ROS work |
| Drive an existing ROS 1 + MoveIt 1 setup | [ROS 1 + MoveIt 1](ros.md) | Legacy stack; only use if you must |
| Just model / visualize a robot cell in a CAD environment | *no backend* | The core data model works without any planner. See [Concepts](../concepts.md) |

## By capability

| Capability | Analytical | Analytical + PyBullet | PyRoKI | PyBullet | ROS 1 + MoveIt 1 | ROS 2 + MoveIt 2 |
|---|:-:|:-:|:-:|:-:|:-:|:-:|
| Forward kinematics | ✓ | ✓ | internal | ✓ | ✓ | ✓ |
| Inverse kinematics | ✓ (closed-form) | ✓ (closed-form) | ✓ (differentiable) | ✓ (numerical) | ✓ | ✓ |
| Collision checking | — | ✓ (mesh-based) | ✓ (approximate) | ✓ (mesh-based) | ✓ | ✓ |
| Point-axis targets | — | — | ✓ | ✓ | ✓ | ✓ |
| Point-to-point motion planning | — | — | — | — | ✓ | ✓ |
| Cartesian motion planning | — | ✓ (partial) | — | ✓ | ✓ | ✓ |
| Visualisation | — | PyBullet GUI | — | PyBullet GUI | RViz | RViz |
| Setup cost | none | `pip install "compas_fab[pybullet]"` | `pip install "compas_fab[pyroki]"` | `pip install "compas_fab[pybullet]"` | Docker | Docker |
| Usable from inside Rhino 8 | ✓ | — | — | — | ✓ (over WebSocket) | ✓ (over WebSocket) |

## Setup cost in plain words

- **Analytical IK**: nothing to install beyond `compas_fab` itself.
- **PyRoKI**: install `compas_fab[pyroki]` in Python 3.10 or newer. The
  dependency is currently pinned to a tested Git commit because it has not
  published a package-index release.
- **PyBullet / Analytical + PyBullet**: one `pip install "compas_fab[pybullet]"` (with a
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
