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

## One-stack run

```bash
# Start the ROS 1 stack
docker compose -f tests/integration_setup/docker-compose.yml up -d

# Run the tests against it
COMPAS_FAB_RUN_ROS_INTEGRATION_TESTS=1 \
COMPAS_FAB_ROS_PORT=9090 \
  uv run pytest tests/backends/ros/test_doc_examples_integration.py
```

Same pattern for ROS 2, swapping the compose file and using
`COMPAS_FAB_ROS_PORT=9091`.

## Parallel run (both distros)

Start both stacks and exercise each from its own pytest invocation:

```bash
# Bring up both stacks
docker compose -f tests/integration_setup/docker-compose.yml up -d
docker compose -f docs/installation/docker_files/ros2-ur10e-demo/docker-compose.yml up -d

# Test against ROS 1
COMPAS_FAB_RUN_ROS_INTEGRATION_TESTS=1 COMPAS_FAB_ROS_PORT=9090 \
  uv run pytest tests/backends/ros/test_doc_examples_integration.py -q

# Test against ROS 2
COMPAS_FAB_RUN_ROS_INTEGRATION_TESTS=1 COMPAS_FAB_ROS_PORT=9091 \
  uv run pytest tests/backends/ros/test_doc_examples_integration.py -q
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

| Var | Meaning | Default |
|---|---|---|
| `COMPAS_FAB_RUN_ROS_INTEGRATION_TESTS` | Set to `1` to opt in; otherwise every test in the file is skipped. | unset |
| `COMPAS_FAB_ROS_HOST` | rosbridge hostname. | `localhost` |
| `COMPAS_FAB_ROS_PORT` | rosbridge port to connect to. | `9090` |
