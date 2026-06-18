# Analytical IK

Closed-form inverse kinematics for known industrial robots. Runs entirely
in-process -no Docker, no PyBullet, no ROS- and is the fastest IK option
in `compas_fab`.

## When to use

- Prototyping reachability and inverse kinematics on a supported robot.
- IK from inside Rhino, Grasshopper or any other Python host.
- Generating IK seeds for a downstream planner.
- You don't need collision checking. (If you do, see
  [Analytical IK + PyBullet](analytical_pybullet.md).)

## Trade-offs

| What you get | What you give up |
|---|---|
| Closed-form IK in microseconds | Only the supported analytical robot families |
| Zero setup beyond `pip install compas_fab` | No collision checking |
| Works from any Python host (CAD, headless, CI) | No motion planning, IK / FK only |

## Setup

Nothing to install beyond `compas_fab` itself. Analytical solvers are part
of the package.

## Supported robots

- Universal Robots: UR3, UR3e, UR5, UR5e, UR10, UR10e, UR16e
- Staubli: TX2-60L
- ABB: IRB 4600 40/255

See [compas_fab.backends][] for the full list of solver classes.

## First example

Forward kinematics on a UR5:

```python
--8<-- "docs/backends/analytical_kinematics/files/01_forward_kinematics.py"
```

## More examples

- [`02_inverse_kinematics.py`](analytical_kinematics/files/02_inverse_kinematics.py){: download="02_inverse_kinematics.py" }: IK returning 8 solutions for a UR5
- [`02_inverse_kinematics with_tools.py`](analytical_kinematics/files/02_inverse_kinematics%20with_tools.py){: download="02_inverse_kinematics with_tools.py" }: IK with a tool attached to the flange

For collision-aware IK and Cartesian planning, see
[Analytical IK + PyBullet](analytical_pybullet.md).

## API reference

- [compas_fab.backends.AnalyticalKinematicsPlanner][]
- [compas_fab.backends][]: all available solvers
