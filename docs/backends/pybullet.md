# PyBullet

[PyBullet](https://pybullet.org/) is a Python module that wraps the Bullet
physics engine. It runs in-process — no Docker, no server — and provides
forward and inverse kinematics, collision checking, and rigid-body dynamics.

`compas_fab` exposes the [PyBulletClient][compas_fab.backends.PyBulletClient]
and [PyBulletPlanner][compas_fab.backends.PyBulletPlanner] classes. The
motion-planning logic itself lives in `compas_fab`, on top of PyBullet's
IK and collision primitives.

## When to use

- You want fast IK and collision checking without standing up ROS.
- You want to simulate multiple robots or kinematic tools in one environment.
- You are running headless (CI, batch planning) or from VS Code.

PyBullet is **not** usable inside Rhino 7 or 8. From Rhino, serialize the
planning problem and run the PyBullet planner from an outside Python process.

## Installation

PyBullet is an optional dependency of `compas_fab`. It is not installed by
default because upstream does not ship wheels for the newest Python versions.

```bash
uv pip install pybullet
```

On macOS, PyBullet sometimes needs an extra build flag:

```bash
CFLAGS="-fno-define-target-os-macros" uv pip install pybullet
```

## Quick start

```pycon
>>> from compas_fab.backends import PyBulletClient, PyBulletPlanner
>>> from compas_fab.robots import RobotCellLibrary
>>> with PyBulletClient(connection_type="direct") as client:
...     planner = PyBulletPlanner(client)
...     cell, state = RobotCellLibrary.ur10e()
...     planner.set_robot_cell(cell, state)
```

Connection types:

- `direct` — headless, no GUI
- `gui` — opens a PyBullet OpenGL window
- `shared_memory` — connect to an external PyBullet server
