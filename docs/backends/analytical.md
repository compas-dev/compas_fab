# Analytical kinematics

For well-known robot geometries (Universal Robots, Staubli) closed-form
inverse kinematics is available without any external dependency. This is
the fastest IK option in `compas_fab` and works fully in-process.

## When to use

- Prototyping reachability and inverse kinematics on a supported robot
- IK from inside Rhino, Grasshopper or any other front-end (no Docker, no PyBullet)
- Generating IK seeds for a downstream planner

Analytical IK does **not** check for collisions. For full motion planning,
chain it with a ROS or PyBullet back-end.

## Supported robots

- Universal Robots: UR3, UR5, UR10, UR3e, UR5e, UR10e, UR16e
- Staubli: TX2-60, TX2-90 (see [API reference](../api/compas_fab.backends.md))

## Quick start

```pycon
>>> from compas_fab.backends import AnalyticalKinematicsPlanner, UR10eKinematics
>>> planner = AnalyticalKinematicsPlanner(UR10eKinematics())
```

See [compas_fab.backends][compas_fab.backends] for the full set of analytical
solvers and the [RobotCellLibrary][compas_fab.robots.RobotCellLibrary] for
ready-made robot cells to plan with.
