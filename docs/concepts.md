# Concepts

**COMPAS FAB** is built on a small data model that is shared across every
backend: you describe **what** you want, then any backend can execute it.
This page walks through the core types. It is backend-agnostic. None of
the code on this page calls a planner.

For planning, see [Choosing a backend](backends/index.md).

## The robot cell

A [`RobotCell`][compas_fab.robots.RobotCell] is the workspace: a robot
model, any tools attached to its flanges, and any rigid bodies (workpieces,
fixtures) that participate in planning.

The [`RobotCellLibrary`][compas_fab.robots.RobotCellLibrary] ships ready-to-use
cells for prototyping:

```pycon
>>> from compas_fab.robots import RobotCellLibrary
>>> cell, state = RobotCellLibrary.ur10e()
>>> print(cell.robot_model)
Robot name=ur10e, Links=11, Joints=10 (6 configurable)
```

Each library entry returns the cell **and** a default state — the second
core type.

## Robot cell state

A [`RobotCellState`][compas_fab.robots.RobotCellState] is a snapshot of the
cell at one moment: the robot's joint
[`Configuration`][compas_robots.Configuration], which tools and bodies are
attached where, and their poses.

A cell tells you *what exists*. A state tells you *where everything is*.
You plan **from** a state and **to** a state (or to a target → next section).

You can mutate state freely between planning calls:

```pycon
>>> state.set_tool_attached_to_group("cone", cell.main_group_name)
>>> state.robot_configuration  # the joints at this moment
```

## Targets and waypoints

A [`Target`][compas_fab.robots.Target] specifies a single goal pose. The
common variants:

- [`FrameTarget`][compas_fab.robots.FrameTarget]: a full Cartesian pose
  for the tool or robot flange.
- [`PointAxisTarget`][compas_fab.robots.PointAxisTarget]: a point plus an
  axis, leaving the rotation around the axis free. Useful for drilling,
  milling, 3D printing, anything with a cylindrical tool.
- [`ConfigurationTarget`][compas_fab.robots.ConfigurationTarget]: joint
  values directly.
- [`ConstraintSetTarget`][compas_fab.robots.ConstraintSetTarget]: for
  region/constraint based targets.

A [`Waypoints`][compas_fab.robots.Waypoints] is a sequence of targets to
follow for Cartesian motion:

- [`FrameWaypoints`][compas_fab.robots.FrameWaypoints]
- [`PointAxisWaypoints`][compas_fab.robots.PointAxisWaypoints]

## Target modes

A target says "go here", but **where** is "here" measured? At the robot's
flange? At the tip of the attached tool? At the workpiece grasped by the
tool? [`TargetMode`][compas_fab.robots.TargetMode] makes that explicit:

- `TargetMode.ROBOT`: the target frame is at the planner coordinate frame
  (PCF) of the robot (typically the `tool0` link).
- `TargetMode.TOOL`: the target is the tool's coordinate frame (TCF).
  The planner does the math to figure out the corresponding PCF.
- `TargetMode.WORKPIECE`: the target is on the attached workpiece. The
  planner accounts for both the tool and the workpiece grasp frame.

```pycon
>>> from compas_fab.robots import FrameTarget, TargetMode
>>> from compas.geometry import Frame
>>> frame = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
>>> target = FrameTarget(frame, TargetMode.TOOL)
```

## Putting it together

A typical planning call is just these four types: a cell, a state, a
target, a planner.

```pycon
>>> # 1. Define what exists
>>> cell, state = RobotCellLibrary.ur5()
>>>
>>> # 2. Define where you want to go
>>> target = FrameTarget(Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0]),
...                      TargetMode.ROBOT)
>>>
>>> # 3. Pick a backend and ask it
>>> # (see Choosing a backend for which line to write here)
>>> # config = planner.inverse_kinematics(target, state)
```

The next step is to [choose a backend](backends/index.md) and run the call.

## Assembling multi-stage motions

A single planning call returns one
[`JointTrajectory`][compas_fab.robots.JointTrajectory]. Real workflows
chain several together, eg. a pick-and-place cycle is six or eight
trajectories interleaved with discrete state changes (gripper closes,
tool attaches, rigid body grasped). An
[`ActionChain`][compas_fab.robots.ActionChain] is the
backend-agnostic data container that holds them together.

A chain threads a single `RobotCellState` through an ordered list of
[`Action`][compas_fab.robots.Action] objects. A single `Action` class
covers both kinds — an action with a trajectory is *planned*, one
without is *unplanned*:

- A **planned (trajectory) action** carries a `JointTrajectory`. The
  chain derives the post-state automatically (last point of the
  trajectory applied to the pre-state).
- A **state-change action** has no trajectory and an explicit
  `post_state`. Use these for discrete transitions: gripper toggle,
  tool attach/detach, rigid body grasped or released.

Every action also carries `tags` (and a free-form `attributes` bag) to
drive differentiated downstream execution — e.g. `approach`/`retract`,
linear vs. free motion.

```pycon
>>> from compas_fab.robots import ActionChain
>>> chain = ActionChain(name="pick_and_place", start_state=state, robot_cell=cell)
>>> chain.append_trajectory("approach",  approach_trajectory, tags=["approach"])
>>> chain.append_trajectory("descend",   descend_trajectory, tags=["linear"])
>>> chain.append_state_change("grasp",   state_with_part_attached)
>>> chain.append_trajectory("retract",   retract_trajectory, tags=["retract"])
>>> chain.duration  # sum of trajectory durations
2.37
>>> chain.end_state  # post-state of the last action
RobotCellState(...)
```

Each `append_*` returns the chain, so chaining works too:

```pycon
>>> chain = (ActionChain(name="pnp", start_state=state, robot_cell=cell)
...        .append_trajectory("approach", t1)
...        .append_state_change("grasp", grasped_state)
...        .append_trajectory("retract", t2))
```

**Serialization.**

Chains can be serialized to/from JSON using COMPAS data serialization mechanism:

```pycon
>>> chain.to_json("pick_and_place.json")
>>> loaded = ActionChain.from_json("pick_and_place.json")
>>> loaded.verify_cell(cell)  # raises if the cell has drifted
```

The chain owns the state sequence so each action's `start_state` (and
its mirror on the contained trajectory) is stripped on serialize and
re-threaded on load: only the chain's own `start_state` and the explicit
state-change post-states are stored. Passing `robot_cell` at construction
records a structural signature (robot name + tool/body ids, hashed with
SHA-256) so a mismatched cell reports a clear error via `verify_cell()`.

For a runnable end-to-end example, see
[`08_motion_plan_pick_and_place.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/pybullet/files/08_motion_plan_pick_and_place.py)
(PyBullet) or [the ROS / MoveIt
equivalent](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/08_motion_plan_pick_and_place.py).

## Where things live

- [`compas_fab.robots`][compas_fab.robots]: every type on this page lives
  here.
- [`compas_fab.backends`][compas_fab.backends]: the planners.
- [`compas_robots`](https://compas.dev/compas_robots/latest/): the
  underlying `RobotModel`, `Configuration`, `Joint`, `Link` types come
  from this lower-level package.
