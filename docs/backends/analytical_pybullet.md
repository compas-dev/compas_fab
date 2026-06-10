# Analytical IK + PyBullet

The hybrid back-end uses closed-form analytical inverse kinematics for speed
and PyBullet underneath for collision checking. It is implemented by
[AnalyticalPyBulletPlanner][compas_fab.backends.AnalyticalPyBulletPlanner]
paired with [AnalyticalPyBulletClient][compas_fab.backends.AnalyticalPyBulletClient].

## When to use

- You need **fast IK** *and* **collision checking** without standing up ROS.
- Your robot is in the analytical solver's supported set (Universal Robots,
  Staubli TX2, ABB IRB4600 40/255).
- You want to filter or rank analytical IK solutions against a real
  collision scene (workpieces, tools, fixtures).

## Trade-offs

| What you get | What you give up |
|---|---|
| Closed-form IK (8 solutions per pose, fast) | Only the supported analytical robot families |
| PyBullet collision checking against tools / workpieces / fixtures | No motion planning, IK only (use `PyBulletPlanner` for that) |
| Runs in-process; no Docker | PyBullet still has to be installed (see [PyBullet setup](pybullet.md#setup)) |

## Setup

```bash
uv pip install pybullet
```

On macOS, PyBullet may need an extra build flag:

```bash
CFLAGS="-fno-define-target-os-macros" uv pip install pybullet
```

The analytical solvers ship with `compas_fab` itself, nothing else to install.

## First example

Compute collision-aware IK for a UR5 against an empty cell:

```python
--8<-- "docs/backends/analytical_kinematics/files/03_iter_ik_pybullet.py"
```

With `keep_order=True`, configurations that fail collision checking are
returned as `None` rather than removed, so the indices stay stable across
calls, useful when feeding adjacent IK calls into a Cartesian path solver.

## More examples

- [`03_analytical_pybullet_planner.py`](analytical_kinematics/files/03_analytical_pybullet_planner.py){: download="03_analytical_pybullet_planner.py" }: full cell with tool attached, workpiece grasp, GUI visualisation
- [`04_cartesian_path_analytic_pybullet.py`](analytical_kinematics/files/04_cartesian_path_analytic_pybullet.py){: download="04_cartesian_path_analytic_pybullet.py" }: Cartesian path planning with collision-aware analytical IK

## API reference

- [compas_fab.backends.AnalyticalPyBulletPlanner][]
- [compas_fab.backends.AnalyticalPyBulletClient][]
- [compas_fab.backends][]: all available analytical solvers
