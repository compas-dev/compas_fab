# PyBullet

[PyBullet](https://pybullet.org/) wraps the Bullet physics engine. It runs
in-process — no Docker, no server — and gives you forward kinematics,
inverse kinematics (numerical), collision checking, and **Cartesian** motion
planning.

!!! note "Free-space `plan_motion` not yet implemented"
    `PyBulletPlanner` does not currently implement free-space
    `plan_motion`. Cartesian motion via `plan_cartesian_motion` works.
    For free-space planning, use one of the ROS backends.

## When to use

- You want fast IK + collision checking without standing up ROS.
- You want to simulate multiple robots or kinematic tools in one scene.
- You are running headless (CI, batch planning) or from VS Code.
- You want a PyBullet GUI to visualize the planning scene.

PyBullet is difficult to install inside Rhino 8+.

## Trade-offs

| What you get | What you give up |
|---|---|
| Numerical IK + collision checking + motion planning | Numerical IK is slower than analytical IK |
| Runs anywhere Python runs (no Docker) | PyBullet has no pre-built wheels for newest Python versions |
| Optional GUI showing the planning scene | Not usable from inside Rhino |

## Setup

PyBullet is an optional dependency of `compas_fab`. It is not installed by
default because upstream does not ship wheels for the newest Python versions.

```bash
uv pip install pybullet
```

On macOS, PyBullet sometimes needs an extra build flag:

```bash
CFLAGS="-fno-define-target-os-macros" uv pip install pybullet
```

`PyBulletClient` supports three connection types:

- `direct` — headless, no GUI (default)
- `gui` — opens a PyBullet OpenGL window
- `shared_memory` — connect to an external PyBullet server

## First example

Inverse + forward kinematics on a UR5 with a GUI:

```python
--8<-- "docs/backends/pybullet/files/04_ik.py"
```

## More examples

Setup:

- [`01_set_robot_cell.py`](pybullet/files/01_set_robot_cell.py){: download="01_set_robot_cell.py" } — building a cell with floor + obstacles
- [`01_set_robot_cell_state.py`](pybullet/files/01_set_robot_cell_state.py){: download="01_set_robot_cell_state.py" } — populating cell state
- [`02_check_collision.py`](pybullet/files/02_check_collision.py){: download="02_check_collision.py" } — collision checking against rigid bodies

Kinematics:

- [`03_fk.py`](pybullet/files/03_fk.py){: download="03_fk.py" }, [`03_fk_target_mode.py`](pybullet/files/03_fk_target_mode.py){: download="03_fk_target_mode.py" }, [`03_fk_to_link.py`](pybullet/files/03_fk_to_link.py){: download="03_fk_to_link.py" } — forward kinematics variants
- [`03_iter_ik.py`](pybullet/files/03_iter_ik.py){: download="03_iter_ik.py" } — iterating over IK solutions
- [`04_ik_point_axis_target.py`](pybullet/files/04_ik_point_axis_target.py){: download="04_ik_point_axis_target.py" } — IK with a free rotation axis (drilling, milling)
- [`04_ik_semi_constrained.py`](pybullet/files/04_ik_semi_constrained.py){: download="04_ik_semi_constrained.py" }, [`04_ik_tool_target_mode.py`](pybullet/files/04_ik_tool_target_mode.py){: download="04_ik_tool_target_mode.py" } — constrained / tool-frame variants
- [`04_ik_errors.py`](pybullet/files/04_ik_errors.py){: download="04_ik_errors.py" }, [`04_ik_multiple_solutions.py`](pybullet/files/04_ik_multiple_solutions.py){: download="04_ik_multiple_solutions.py" } — error handling and multi-solution patterns

Motion planning:

- [`06_cartesian_motion_frame_waypoints.py`](pybullet/files/06_cartesian_motion_frame_waypoints.py){: download="06_cartesian_motion_frame_waypoints.py" } — Cartesian path with `FrameWaypoints`
- [`06_cartesian_motion_point_axis_waypoints.py`](pybullet/files/06_cartesian_motion_point_axis_waypoints.py){: download="06_cartesian_motion_point_axis_waypoints.py" } — Cartesian path with `PointAxisWaypoints`
- [`08_motion_plan_pick_and_place.py`](pybullet/files/08_motion_plan_pick_and_place.py){: download="08_motion_plan_pick_and_place.py" } — assembling several trajectories + state changes into a serializable `ActionChain` (see [Concepts](../concepts.md#assembling-multi-stage-motions))

## API reference

- [compas_fab.backends.PyBulletClient][]
- [compas_fab.backends.PyBulletPlanner][]
- [compas_fab.backends.pybullet.backend_features][] — the underlying feature implementations
