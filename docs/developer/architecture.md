# Backend client architecture

This document details the architecture used to implement backend clients and
backend features. It is aimed at contributors extending `compas_fab` with a
new backend — it is not needed for everyday planning. End users interact with
the planner and client classes in
[compas_fab.backends][compas_fab.backends]; each planner already exposes every
backend-feature method directly, composing the features below through multiple
inheritance.

## Clients vs. planners

To keep backends consistent and modular, `compas_fab` defines two interfaces:

- [ClientInterface][compas_fab.backends.interfaces.ClientInterface] —
  connection lifecycle and shared state (connect, disconnect, etc.).
- [PlannerInterface][compas_fab.backends.interfaces.PlannerInterface] —
  planning, scene management, and kinematics.

Any new backend should inherit from `ClientInterface` and pair it with a
`PlannerInterface` subclass.

> Eventually, execution and control will move to a separate `ControlInterface`;
> for now those methods sit on the client.

## Backend features

`PlannerInterface` provides default behavior for each planning method.
To override a default, implement the appropriate **backend feature**
interface from
[compas_fab.backends.interfaces][compas_fab.backends.interfaces].

Backend feature classes are **callable**: their `__call__` magic method
delegates to the named method. For example:

```python
from compas.geometry import Frame
from compas_fab.backends.interfaces import InverseKinematics

class ExampleInverseKinematics(InverseKinematics):
    def inverse_kinematics(self, robot, frame_WCF,
                           start_configuration=None,
                           group=None,
                           options=None):
        ...

calculate_example_ik = ExampleInverseKinematics()
frame = Frame([0, 0, 0], [1, 0, 0], [0, 1, 0])
ik_result = calculate_example_ik(robot, frame)
# equivalent to:
ik_result = calculate_example_ik.inverse_kinematics(robot, frame)
```

The feature interfaces enforce a common signature across all
implementations. Adhere to the documented types for arguments and
return values.

## Mixing features from multiple backends

The feature interfaces are designed so a user can mix and match features
from different backends. For example, if `ClientA` is fast at inverse
kinematics but lacks motion planning, and `ClientB` is slow at IK but
plans well:

```python
with ClientA() as client_a, ClientB() as client_b:
    inverse_kinematics = ClientAInverseKinematics(client_a)
    plan_motion = ClientBPlanMotion(client_b)
```

## Implemented backend features

The following backend feature modules are provided:

- [compas_fab.backends.interfaces][compas_fab.backends.interfaces] —
  the abstract base classes.
- [compas_fab.backends.ros.backend_features][compas_fab.backends.ros.backend_features] —
  ROS / MoveIt implementations.
- [compas_fab.backends.pybullet.backend_features][compas_fab.backends.pybullet.backend_features] —
  PyBullet implementations.
- [compas_fab.backends.kinematics.backend_features][compas_fab.backends.kinematics.backend_features] —
  analytical-kinematics implementations.
