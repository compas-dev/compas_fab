# CAD front-ends

`compas_fab` is CAD-independent. The core API works in any Python
environment; host bindings are provided for several CAD environments and
viewers.

You only need to set up the front-end you actually use. The planning
backend ([choose one](backends/index.md)) is a separate decision.

## Rhino 8

`compas_fab` is compatible with Rhino 8 and later. Add the requirement
header to the top of your script in the Rhino 8 Script Editor:

```python
# r: compas_fab
```

Rhino 8 ships with CPython 3.9, which is why `compas_fab` keeps 3.9 as its
minimum Python version.

## Grasshopper

The Grasshopper components are bundled with `compas_fab`. After installing
in a Rhino 8-compatible environment, the components appear in the
Grasshopper toolbar.

Grasshopper examples that ship with the repo:

- [`gh_forward_kinematics.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/gh_forward_kinematics.py)
- [`gh_inverse_kinematics.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/gh_inverse_kinematics.py)
- [`gh_plan_motion.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/gh_plan_motion.py)
- [`gh_plan_cartesian_motion.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/gh_plan_cartesian_motion.py)
- [`gh_robot_visualisation.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/gh_robot_visualisation.py)

For developing the components, see
[Developer guide › Grasshopper components](developer/grasshopper.md).

## Blender

Install `compas_fab` into Blender's bundled Python. See the
[COMPAS Blender installation guide](https://compas.dev/compas/latest/userguide/cad.blender.html)
for setting up Blender's Python, then run `pip install compas_fab` inside
that interpreter.

## COMPAS Viewer

The COMPAS viewer integration is available out of the box once
`compas_viewer` is installed in the same environment:

```bash
uv pip install compas_viewer
```

Robot scene objects come from
[`compas_robots.viewer`](https://compas.dev/compas_robots/latest/api/compas_robots.viewer/);
`compas_fab` itself does not ship a separate viewer module.

## Headless / VS Code

`compas_fab` is fully usable from a plain Python script without any CAD
host — useful for headless planning, batch jobs, CI, and PyBullet GUIs.

This is the recommended environment for the
[PyBullet](backends/pybullet.md), [Analytical + PyBullet](backends/analytical_pybullet.md)
and ROS-based backends, none of which run inside Rhino's Python.

## Which front-end works with which backend?

| Front-end | Analytical | Analytical + PyBullet | PyBullet | ROS / ROS 2 |
|---|:-:|:-:|:-:|:-:|
| Rhino 8 | ✓ | — | — | ✓ (over WebSocket) |
| Grasshopper | ✓ | — | — | ✓ (over WebSocket) |
| Blender | ✓ | ✓ | ✓ | ✓ |
| COMPAS Viewer | ✓ | ✓ | ✓ | ✓ |
| Headless / VS Code | ✓ | ✓ | ✓ | ✓ |

PyBullet (and the Analytical + PyBullet hybrid) cannot run inside Rhino's
Python because PyBullet ships native binaries that Rhino's interpreter
cannot load. From Rhino, serialise the planning problem and run PyBullet
from an outside Python process.
