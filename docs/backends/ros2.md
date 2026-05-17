# ROS 2 / MoveIt 2

The current ROS-based planning back-end for `compas_fab`. Uses ROS 2 Jazzy
(LTS, 2024–2029), MoveIt 2, and `rmw_zenoh_cpp` for transport (replacing the
DDS discovery layer with a Zenoh router).

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

## Quick start (UR10e demo)

```bash
cd docs/installation/docker_files/ros2-ur10e-demo
docker compose up
```

This starts seven services:

| Service | Purpose | Port |
|---|---|---|
| `ur-sim` | URSim simulator (UR10e) | 5900 (VNC), 6080 (noVNC pendant) |
| `zenoh-router` | Zenoh router federating ROS 2 traffic | — |
| `ur-driver` | UR ROS 2 driver | — |
| `moveit-demo` | MoveIt 2 planning + RViz | — |
| `ros-bridge` | rosbridge WebSocket | 9090 |
| `file-server` | HTTP server for URDF/mesh assets | 9091 |
| `gui` | noVNC web X11 server (RViz viewport) | 8080 |

Open the **simulated teach pendant** at <http://localhost:6080/vnc.html> and
power on the robot (Initialise → Brake release → Play) before issuing motion
commands.

Open the **RViz viewport** at <http://localhost:8080/vnc.html> to see the
planning scene rendered from the `moveit-demo` container.

## Connecting from Python

```python
from compas_fab.backends import RosClient

with RosClient(host="localhost", port=9090) as ros:
    print(ros.is_connected)
```

## Loading robot descriptions over HTTP

When using ROS 2 we recommend [HttpFileServerLoader][compas_fab.backends.HttpFileServerLoader]
to load URDFs and mesh files from the `file-server` container, instead of the
legacy `RosFileServerLoader` (which depends on a ROS 1 service):

```python
from compas_fab.backends import HttpFileServerLoader

loader = HttpFileServerLoader(base_url="http://localhost:9091")
urdf = loader.load_urdf("/robot_description", "std_msgs/String")
```

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
