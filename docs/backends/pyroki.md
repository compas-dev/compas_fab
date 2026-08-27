# PyRoKI

[PyRoKI](https://github.com/chungmin99/pyroki) provides differentiable robot
kinematics on JAX. The COMPAS FAB backend converts a `RobotModel` directly into
PyRoKI's kinematic representation and exposes it through the usual
`PyRokiPlanner.inverse_kinematics()` API.

## When to use

- You need fast warm-start numerical IK for servoing or interactive design.
- Your robot has redundant joints, external axes, or no analytical solver.
- You need `FrameTarget` or `PointAxisTarget` constraints.
- You want collision-aware IK without running ROS or a separate service.

## Trade-offs

| What you get | What you give up |
|---|---|
| In-process differentiable IK with inexpensive warm solves | The first solve compiles a JAX problem and is comparatively slow |
| Arbitrary COMPAS robot models and planning groups | One solution is currently produced per starting configuration |
| Collision checking and avoidance enabled by default | Collision meshes are approximated, not checked exactly |
| Joint limits, tools, workpieces, rigid bodies, and target modes | No point-to-point or Cartesian trajectory planning yet |

## Setup

PyRoKI is an optional dependency and requires Python 3.10 or newer. Python 3.12
is the version exercised by COMPAS FAB's PyRoKI CI job.

```bash
uv pip install "compas_fab[pyroki]"
```

PyRoKI has not published a package-index release. The extra therefore installs
the exact Git commit recorded in `requirements-pyroki.txt`. Updating that pin
should be accompanied by the complete PyRoKI test suite because the upstream
interfaces can still change before its first release.

## First example

Solve a frame target and verify it independently with COMPAS Robots forward
kinematics:

```python
--8<-- "docs/backends/pyroki/files/01_inverse_kinematics.py"
```

This example loads no meshes, so it explicitly sets `check_collision=False`.
Collision checking is enabled by default for normal geometry-loaded cells.

## Collision model

The backend preserves FAB's disabled-collision pairs, hidden objects,
attachments, `touch_links`, and `touch_bodies`. Each moving collision mesh is
approximated by a fitted capsule, while each stationary world mesh is
approximated by an axis-aligned bounding box. This makes distances
differentiable and solves fast, but it can produce false positive or false
negative results compared with a mesh-based checker. Validate safety-critical
results with a higher-fidelity backend such as PyBullet or MoveIt.

`collision_margin` adds clearance around the approximations. Passing
`options={"check_collision": False}` opts out of collision constraints and
allows a cell loaded without collision geometry.

## Warm starts and caching

The configuration in `RobotCellState` is the numerical seed. The first query
for a robot cell and solver structure includes JAX compilation and collision
geometry setup. The planner caches that analyzed problem, so later targets and
active-joint seed values with the same structure are inexpensive. Changes to
the cell, attachments, stationary geometry, inactive joints, planning group,
target kind, or structural solver options select a new cache entry.

For continuous servoing, keep one planner instance, merge each returned
configuration into the state, and use that state as the next query's seed.

## Options

| Option | Default | Meaning |
|---|---:|---|
| `check_collision` | `True` | Include collision constraints and verify the result |
| `collision_margin` | `0.0` | Additional clearance in metres |
| `collision_weight` | `10.0` | Weight of collision residuals |
| `max_iterations` | `100` | Maximum nonlinear solver iterations |
| `lambda_initial` | `1.0` | Initial trust-region damping |
| `position_weight` | `50.0` | Target-position residual weight |
| `orientation_weight` | `10.0` | Target-orientation or target-axis residual weight |
| `position_tolerance` | `1e-4` | Accepted position error in metres, unless set on the target |
| `orientation_tolerance` | `1e-3` | Accepted angular error in radians, unless set on the target |
| `return_full_configuration` | `False` | Return all configurable joints instead of only the planning group |
| `verbose` | `False` | Enable PyRoKI/JAXLS solver output |

## More examples

- [`02_collision_aware_inverse_kinematics.py`](pyroki/files/02_collision_aware_inverse_kinematics.py){: download="02_collision_aware_inverse_kinematics.py" } — move a redundant Panda posture away from an obstacle while preserving its end-effector pose
- [`03_point_axis_target.py`](pyroki/files/03_point_axis_target.py){: download="03_point_axis_target.py" } — constrain position and tool axis while leaving rotation about that axis free
- [`04_warm_start_servo.py`](pyroki/files/04_warm_start_servo.py){: download="04_warm_start_servo.py" } — repeatedly update a target and feed each solution into the next warm start

## API reference

- [compas_fab.backends.PyRokiPlanner][]
- [compas_fab.backends.pyroki.backend_features][]
