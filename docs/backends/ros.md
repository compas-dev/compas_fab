# ROS 1 / MoveIt 1

The ROS 1 / MoveIt 1 back-end is the legacy planning stack for `compas_fab`.
For new projects we recommend the [ROS 2 / MoveIt 2](ros2.md) back-end —
ROS 1 reached end of life on Noetic and receives no further upstream updates.

## When to use

- You already have a ROS 1 workspace or a robot that only ships ROS 1 drivers.
- You are reproducing a legacy `compas_fab` example or paper.
- For anything new, prefer [ROS 2 / MoveIt 2](ros2.md).

## Trade-offs

| What you get | What you give up |
|---|---|
| Full motion planning via MoveIt (with collision checking, trajectory smoothing) | Requires Docker + a per-robot compose stack |
| Per-robot demo stacks shipped with the repo | ROS 1 is EOL — no new features, only bug fixes |
| Works from any host OS (Docker Desktop on Mac/Windows is enough) | Slower iteration than in-process backends like PyBullet |

## Architecture

`compas_fab` talks to ROS over
[`rosbridge`](https://github.com/RobotWebTools/rosbridge_suite), a
WebSocket bridge that translates JSON messages to ROS topics, services and
actions. The back-end runs inside Docker; your Python code (in any
front-end) connects to it as a client.

- [RosClient][compas_fab.backends.RosClient]: low-level rosbridge connection
- [MoveItPlanner][compas_fab.backends.MoveItPlanner]: planning service wrapper

## Setup

Per-robot Docker compose stacks live under
[`docs/installation/docker_files/`](https://github.com/compas-dev/compas_fab/tree/main/docs/installation/docker_files).
Each demo bundles ROS 1, MoveIt 1, a robot driver and `rosbridge_server`.

Start, for example, the UR10e demo:

```bash
cd docs/installation/docker_files/ur10e-demo
docker compose up
```

## First example

Connect from Python and verify the bridge:

```python
--8<-- "docs/backends/ros/files/01_ros_connection.py"
```

## More examples

Setup:

- [`02_load_robot_cell.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/02_load_robot_cell.py) — load the robot cell from ROS
- [`02_set_robot_cell.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/02_set_robot_cell.py), [`02_set_robot_cell_state_attach_objects.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/02_set_robot_cell_state_attach_objects.py), [`02_set_robot_cell_state_with_kinematic_tools.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/02_set_robot_cell_state_with_kinematic_tools.py) — populating cells and states

Kinematics:

- [`03_forward_kinematics.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/03_forward_kinematics.py), [`03_forward_kinematics_target_mode.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/03_forward_kinematics_target_mode.py), [`03_forward_kinematics_to_link.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/03_forward_kinematics_to_link.py) — FK variants
- [`04_ik.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/04_ik.py), [`04_ik_target_mode.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/04_ik_target_mode.py), [`04_ik_allow_collision.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/04_ik_allow_collision.py), [`04_ik_full_config.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/04_ik_full_config.py), [`04_ik_unreachable.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/04_ik_unreachable.py), [`04_iter_ik.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/04_iter_ik.py) — IK variants
- [`04_ik_cache.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/04_ik_cache.py), [`04_iter_ik_unique.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/04_iter_ik_unique.py) — IK caching and unique-solution filtering

Motion planning:

- [`05_plan_motion.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/05_plan_motion.py), [`05_plan_motion_configuration.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/05_plan_motion_configuration.py), [`05_plan_motion_tolerance.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/05_plan_motion_tolerance.py) — point-to-point motion planning
- [`05_plan_motion_with_attachment.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/05_plan_motion_with_attachment.py), [`05_plan_motion_with_obstacle.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/05_plan_motion_with_obstacle.py) — planning with attached tools or obstacles
- [`06_cartesian_motion.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/06_cartesian_motion.py), [`06_cartesian_motion_partial.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/06_cartesian_motion_partial.py), [`06_cartesian_motion_step_size.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/06_cartesian_motion_step_size.py), [`06_cartesian_motion_target_mode.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/06_cartesian_motion_target_mode.py) — Cartesian motion variants
- [`08_motion_plan_pick_and_place.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/08_motion_plan_pick_and_place.py) — assembling several trajectories + state changes into a serializable `ActionChain` (see [Concepts](../concepts.md#assembling-multi-stage-motions))

Topics / pubsub:

- [`10_ros_hello_world_talker.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/10_ros_hello_world_talker.py), [`10_ros_hello_world_listener.py`](https://github.com/compas-dev/compas_fab/blob/main/docs/backends/ros/files/10_ros_hello_world_listener.py) — raw ROS pub/sub via rosbridge

## API reference

- [compas_fab.backends.RosClient][]
- [compas_fab.backends.MoveItPlanner][]
- [compas_fab.backends.RosFileServerLoader][]
- [compas_fab.backends.ros.backend_features][] — the underlying feature implementations
