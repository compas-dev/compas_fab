# ROS 2 / MoveIt 2

The current ROS-based planning back-end for `compas_fab`. Uses ROS 2 Jazzy
(LTS, 2024–2029), MoveIt 2, and `rmw_zenoh_cpp` for transport (replacing
DDS multicast discovery with a Zenoh router).

## When to use

- You want full motion planning (collision checking, trajectory smoothing)
  against a real or simulated robot.
- You are starting a new project — ROS 2 Jazzy is the supported, current
  ROS LTS.
- You need to drive the robot from any host OS (Docker Desktop on Mac /
  Windows is enough).

## Trade-offs

| What you get | What you give up |
|---|---|
| Full MoveIt 2 motion planning + URSim simulator | Docker required (heavier startup than in-process backends) |
| Zenoh transport avoids DDS multicast issues on Docker bridges | Per-robot compose stacks need maintenance |
| Web-based RViz viewport + simulated teach pendant | First-time setup is involved (see below) |

## Architecture

```
┌─────────────────┐    rosbridge       ┌─────────────────┐
│  compas_fab     │ ◄────WebSocket───► │  ros-bridge     │
│  (your script)  │     (port 9090)    │                 │
└─────────────────┘                    └────────┬────────┘
                                                │
                            ┌───────────────────┼───────────────────┐
                            │                   │                   │
                       ┌────▼─────┐      ┌──────▼─────┐      ┌──────▼──────┐
                       │ ur-driver│      │ moveit-demo│      │  ur-sim     │
                       │          │      │  (RViz)    │      │ (URSim)     │
                       └────┬─────┘      └──────┬─────┘      └──────┬──────┘
                            │                   │                   │
                            └─────────┬─────────┴───────────────────┘
                                      │
                              ┌───────▼────────┐
                              │ zenoh-router   │
                              └────────────────┘
```

All ROS 2 nodes are Zenoh clients pointing at a single `zenoh-router` service
that federates traffic between containers. This eliminates the need for DDS
multicast discovery, which is unreliable on Docker bridges and across
host-OS boundaries.

## Setup

Two compose files ship with the project, sharing the same Dockerfile and
image (`compas-fab/ros-jazzy-moveit2`):

### Lightweight test stack

For integration tests and headless service calls: just MoveIt + rosbridge
+ file server, no robot driver, no simulator, no GUI.

```bash
docker compose -f tests/integration_setup/docker-compose-ros2.yml up
```

| Service | Purpose | Port |
|---|---|---|
| `zenoh-router` | Zenoh router federating ROS 2 traffic | — |
| `moveit2-demo` | MoveIt 2 planning node | — |
| `ros2-bridge` | rosbridge WebSocket | 9091 |
| `file-server` | HTTP server for URDF/mesh assets | 9092 |

### Full demo stack (URSim + GUI)

For end-to-end demos. It adds URSim, the real UR ROS 2 driver, and a
noVNC RViz viewport on top of the test stack.

```bash
docker compose -f docs/installation/docker_files/ros2-ur10e-demo/docker-compose.yml up
```

This adds:

| Service | Purpose | Port |
|---|---|---|
| `ur-sim` | URSim simulator (UR5) | 5900 (VNC), 6080 (noVNC pendant) |
| `ur-driver` | UR ROS 2 driver | — |
| `gui` | noVNC web X11 server for RViz | 8080 |

Open the **simulated teach pendant** at <http://localhost:6080/vnc.html>
and power on the robot (Initialise → Brake release → Play) before issuing
motion commands.

Open the **RViz viewport** at <http://localhost:8080/vnc.html> to see the
planning scene rendered from the `moveit2-demo` container.

## First example

Connect from Python and verify the bridge:

```python
--8<-- "docs/backends/ros/files/01_ros_connection.py"
```

## Loading robot descriptions over HTTP

For ROS 2 we recommend
[HttpFileServerLoader][compas_fab.backends.HttpFileServerLoader] to load
URDFs and meshes from the `file-server` container, instead of the legacy
`RosFileServerLoader` (which depends on a ROS 1 service):

```python
from compas_fab.backends import HttpFileServerLoader

loader = HttpFileServerLoader(base_url="http://localhost:9091")
urdf = loader.load_urdf("/robot_description", "std_msgs/String")
```

## More examples

The example scripts in `docs/backends/ros/files/` target both ROS 1 and ROS 2
back-ends because the `RosClient` API is unchanged — see the
[ROS / MoveIt 1 examples](ros.md#more-examples) for the full list (kinematics,
motion planning, Cartesian motion, pub/sub).

## Notes on Zenoh

- All ROS 2 services in this stack run as Zenoh clients (not the default DDS).
- The session config lives at
  [`zenoh_session.json5`](https://github.com/compas-dev/compas_fab/tree/main/docs/installation/docker_files/ros2-ur10e-demo/zenoh_session.json5).
- POSIX shared-memory transport is **disabled** in that config because it
  fails under Rosetta 2 on Apple Silicon and is unnecessary for TCP-only
  inter-container traffic.
- `ZENOH_SESSION_CONFIG_URI` takes a **plain filesystem path** despite the name,
  not a `file://` URI.

## Notes on controller rate

The UR driver defaults to a 500 Hz control loop. Inside Docker this can cause
overrun warnings. The demo lowers this to 100 Hz via `update_rate.yaml`
mounted at `/etc/compas_fab/update_rate.yaml` and passed to the launch file
with `update_rate_config_file:=...`.

## Architecture: amd64 only

URSim and the noVNC X11 server are amd64-only. On Apple Silicon they run
through Rosetta 2; `platform: linux/amd64` is set on those services.

## API reference

- [compas_fab.backends.RosClient][]
- [compas_fab.backends.MoveItPlanner][]
- [compas_fab.backends.HttpFileServerLoader][]
- [compas_fab.backends.ros.backend_features][] — the underlying feature implementations
