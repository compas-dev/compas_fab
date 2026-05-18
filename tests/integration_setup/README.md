# Integration test setup

The ROS-backed integration tests in
[`tests/backends/ros/test_doc_examples_integration.py`](../backends/ros/test_doc_examples_integration.py)
talk to a real rosbridge over WebSocket. Both ROS 1 (Noetic + MoveIt 1)
and ROS 2 (Jazzy + MoveIt 2) are supported — the test fixture
auto-detects the distro from `client.ros_distro` and adapts.

## Stacks

| Stack | Compose file | rosbridge port | other ports |
|---|---|---|---|
| **ROS 1** (Noetic + MoveIt 1, UR5) | [`tests/integration_setup/docker-compose.yml`](docker-compose.yml) | **9090** | `11311` (rosmaster) |
| **ROS 2** (Jazzy + MoveIt 2, UR5)  | [`docs/installation/docker_files/ros2-ur10e-demo/docker-compose.yml`](../../docs/installation/docker_files/ros2-ur10e-demo/docker-compose.yml) | **9091** | `9092` (HTTP assets), `8080` (noVNC), `5900`/`6080` (URSim) |

Defaults don't collide, so both stacks can run simultaneously and the
integration tests can target either one.

## Test fixtures

Two fixtures live in [`tests/backends/ros/conftest.py`](../backends/ros/conftest.py),
one per stack. A test opts into the stack it cares about:

```python
def test_something_ros1(ros1_client):
    ...

def test_something_ros2(ros2_client):
    ...
```

Each fixture skips independently if its stack isn't reachable, so the
suite degrades gracefully when only one stack is up.

The integration tests currently in
[`test_doc_examples_integration.py`](../backends/ros/test_doc_examples_integration.py)
assert ROS-2-specific URDF / SRDF shapes (`world` root link,
`ur_manipulator` planning group) and therefore depend on `ros2_client`.

## One-stack run

```bash
# Start the ROS 2 stack
docker compose -f docs/installation/docker_files/ros2-ur10e-demo/docker-compose.yml up -d

# Run the tests against it (default port 9091 — no env var needed)
COMPAS_FAB_RUN_ROS_INTEGRATION_TESTS=1 \
  uv run pytest tests/backends/ros/test_doc_examples_integration.py
```

## Parallel run (both stacks)

Bring up both, then invoke pytest once:

```bash
# Both stacks
docker compose -f tests/integration_setup/docker-compose.yml up -d
docker compose -f docs/installation/docker_files/ros2-ur10e-demo/docker-compose.yml up -d

# All tests; each one runs against the stack its fixture targets,
# skipping if that stack isn't reachable.
COMPAS_FAB_RUN_ROS_INTEGRATION_TESTS=1 \
  uv run pytest tests/backends/ros/ -q
```

## Overriding ports

Every host-side port is `${VAR:-DEFAULT}`-driven so you can remap if
something on the host already holds the default:

| Var | Stack | Default | Container port |
|---|---|---|---|
| `ROS1_BRIDGE_PORT` | ROS 1 | 9090 | 9090 |
| `ROS1_CORE_PORT`   | ROS 1 | 11311 | 11311 |
| `ROS2_BRIDGE_PORT` | ROS 2 | 9091 | 9090 |
| `ROS2_HTTP_PORT`   | ROS 2 | 9092 | 9091 |

For example, to run the ROS 2 rosbridge on the traditional 9090:

```bash
ROS2_BRIDGE_PORT=9090 docker compose -f docs/installation/docker_files/ros2-ur10e-demo/docker-compose.yml up -d
COMPAS_FAB_RUN_ROS_INTEGRATION_TESTS=1 COMPAS_FAB_ROS_PORT=9090 \
  uv run pytest tests/backends/ros/test_doc_examples_integration.py
```

## Test fixture environment variables

| Var | Used by | Default |
|---|---|---|
| `COMPAS_FAB_RUN_ROS_INTEGRATION_TESTS` | both fixtures | unset → all skip |
| `COMPAS_FAB_ROS_HOST` | both fixtures | `localhost` |
| `COMPAS_FAB_ROS1_PORT` | `ros1_client` | `9090` |
| `COMPAS_FAB_ROS2_PORT` | `ros2_client` | `9091` |
| `COMPAS_FAB_ROS_PORT` *(legacy)* | falls back as `ros1_client` port if set | unset |

Overriding ROS 2 to talk on the legacy 9090 port (e.g. after running
`ROS2_BRIDGE_PORT=9090 docker compose ... up`):

```bash
COMPAS_FAB_RUN_ROS_INTEGRATION_TESTS=1 COMPAS_FAB_ROS2_PORT=9090 \
  uv run pytest tests/backends/ros/test_doc_examples_integration.py
```
