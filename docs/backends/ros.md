# ROS / MoveIt 1

The ROS 1 / MoveIt 1 back-end is the legacy planning stack for `compas_fab`.
For new projects we recommend the [ROS 2 / MoveIt 2](ros2.md) back-end —
ROS 1 reached end of life on Noetic and receives no further upstream updates.

## Architecture

`compas_fab` talks to ROS over [`rosbridge`](https://github.com/RobotWebTools/rosbridge_suite),
a WebSocket bridge that translates JSON messages to ROS topics, services and
actions. The back-end runs inside Docker; your Python code (in any front-end)
connects to it as a client.

- [RosClient][compas_fab.backends.RosClient]: low-level rosbridge connection
- [MoveItPlanner][compas_fab.backends.MoveItPlanner]: planning service wrapper

## Docker images

Per-robot Docker compose stacks live under
[`docs/installation/docker_files/`](https://github.com/compas-dev/compas_fab/tree/main/docs/installation/docker_files).
Each demo bundles ROS 1, MoveIt 1, a robot driver and `rosbridge_server`.

To start, for example, the UR10e demo:

```bash
cd docs/installation/docker_files/ur10e-demo
docker compose up
```

Then connect from Python:

```python
from compas_fab.backends import RosClient

with RosClient(host="localhost", port=9090) as ros:
    print(ros.is_connected)
```

## Limitations

- ROS 1 only ships on Ubuntu 20.04 (Noetic). Newer hosts run it through Docker.
- No new features upstream — bug fixes only.
- Migration to ROS 2 is recommended for any new work.
