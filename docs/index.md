# COMPAS FAB

**Robotic fabrication for the COMPAS Framework.** One library that drives
five planning backends — from microsecond closed-form IK to full ROS 2 /
MoveIt 2 motion planning — and works from any CAD environment or a plain
Python script.

```pycon
>>> from compas_fab.robots import RobotCellLibrary
>>> cell, state = RobotCellLibrary.ur10e()
>>> print(cell.robot_model)
Robot name=ur10e, Links=11, Joints=10 (6 configurable)
```

## The five backends

| Backend | What it does | Setup |
|---|---|---|
| [Analytical IK](backends/analytical.md) | Closed-form IK on UR, Staubli, ABB IRB4600 | None |
| [Analytical IK + PyBullet](backends/analytical_pybullet.md) | Analytical IK with PyBullet collision checking | `pip install pybullet` |
| [PyBullet](backends/pybullet.md) | Numerical IK, collision checking, motion planning, in-process | `pip install pybullet` |
| [ROS / MoveIt 1](backends/ros.md) | Full motion planning over rosbridge (legacy) | Docker |
| [ROS 2 / MoveIt 2](backends/ros2.md) | Full motion planning over rosbridge (current) | Docker |

Not sure which fits? See **[Choosing a backend](backends/index.md)**.

## I want to…

- **…just compute IK on a UR/Staubli/ABB** → [Analytical IK](backends/analytical.md)
- **…compute IK and check collisions, no Docker** → [Analytical IK + PyBullet](backends/analytical_pybullet.md) or [PyBullet](backends/pybullet.md)
- **…plan full motion against a UR over ROS 2** → [ROS 2 / MoveIt 2](backends/ros2.md)
- **…drive a robot from Rhino or Grasshopper** → [CAD front-ends](frontends.md)
- **…just understand the data model** → [Concepts](concepts.md)

## What's next

1. **[Install compas_fab](installation.md)** — `uv pip install compas_fab`
   plus the optional pieces you need.
2. **[Set up a CAD front-end](frontends.md)** *(if you want one)* — Rhino,
   Grasshopper, Blender, COMPAS Viewer, or just headless Python.
3. **[Choose a backend](backends/index.md)** — pick a planner and follow its
   quick start.
4. **[Concepts](concepts.md)** — the backend-agnostic data model
   (`RobotCell`, `RobotCellState`, `Target`, `Waypoints`).
5. **[API reference](api/index.md)** — the full module documentation.

## How it fits with the COMPAS framework

`compas_fab` builds on [COMPAS](https://compas.dev/) and
[COMPAS Robots](https://compas.dev/compas_robots/latest/). The robot
model, kinematic chain and URDF loaders come from `compas_robots`;
`compas_fab` adds planning, execution and CAD bindings on top.
