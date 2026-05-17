# ROS 2 + MoveIt 2 + UR10e demo

Reference backend for testing `compas_fab` against ROS 2 with MoveIt 2 and a simulated UR10e.

## Stack

- **ROS 2 Jazzy** (LTS, May 2024 – May 2029)
- **MoveIt 2** (`ros-jazzy-moveit`)
- **Universal Robots ROS 2 driver** (`ros-jazzy-ur`, `ros-jazzy-ur-moveit-config`) connected to the official URSim Polyscope simulator
- **rmw_zenoh_cpp** as RMW (replaces DDS): a single `zenoh-router` service federates traffic; no multicast discovery needed
- **rosbridge_suite** for WebSocket access from `roslibpy` / `compas_fab` on port `9090`

## Run

```bash
docker compose up --build
```

First run builds the image (`compas-fab/ros-jazzy-moveit2-ur10e`) and pulls URSim / noVNC; subsequent runs reuse them. URSim takes ~30–60 seconds to fully boot — the driver's retry logic handles the cold start.

Open in a browser:

- `http://localhost:8080/vnc.html` — RViz (planning scene, motion previews)
- `http://localhost:6080/vnc.html` — Simulated UR teach pendant (Polyscope)

`roslibpy` / `compas_fab` connects to `ws://localhost:9090`; meshes referenced as `package://...` come from `http://localhost:9091/...`.

## Services

| Service       | Port                     | Purpose                                                                                            |
| ------------- | ------------------------ | -------------------------------------------------------------------------------------------------- |
| `ur-sim`        | 5900, 6080             | Official UR Polyscope simulator (`universalrobots/ursim_e-series`)                                 |
| `zenoh-router`  | —                      | `rmw_zenohd` — federates traffic for all ROS 2 nodes (replaces DDS discovery)                      |
| `ur-driver`     | —                      | `ur_control.launch.py` — real `ur_robot_driver` connected to `ur-sim`                              |
| `moveit-demo` | —                        | `ur_moveit.launch.py` — `move_group`, planning scene, RViz                                         |
| `ros-bridge`  | 9090                     | rosbridge WebSocket for `roslibpy` / `compas_fab`                                                  |
| `file-server` | 9091                     | Plain HTTP server over `/opt/ros/jazzy/share/` for meshes                                          |
| `gui`         | 8080                     | `theasp/novnc` — shared X11 server; serves RViz via the browser                                    |

### Why `ur-sim` instead of `use_fake_hardware`

`use_fake_hardware:=true` only mocks `ros2_control`'s hardware interface. The driver's auxiliary nodes (`urscript_interface`, `robot_state_helper`, `dashboard_client`) still TCP-connect to the UR primary / secondary / dashboard ports and spam the log on failure. Running the official URSim image gives the driver an actual Polyscope controller to talk to — same protocol as a real arm, no mocking.

### First-time setup on the Polyscope pendant

URSim ships with the ExternalControl URCap pre-installed and a sample program loaded. After `docker compose up`, open `http://localhost:6080/vnc.html` and on the simulated pendant:

1. Tap the red robot button (bottom-left of the pendant) → **Power on** → **Start**. The status bar should change from `Power Off` → `Idle` → `Normal`.
2. Tap **Play** (bottom triangle) to start the ExternalControl program. The driver will pick up the connection and begin streaming joint states.

### Zenoh transport (instead of DDS)

The stack uses `rmw_zenoh_cpp` rather than the default DDS RMW. All ROS services have:

- `RMW_IMPLEMENTATION=rmw_zenoh_cpp`
- `ZENOH_SESSION_CONFIG_URI=file:///etc/zenoh/session.json5` — points each client at the `zenoh-router` service at `tcp/zenoh-router:7447`
- `ZENOH_ROUTER_CHECK_ATTEMPTS=-1` — wait forever for the router rather than failing fast

The session config is baked into the image via the Dockerfile (`COPY zenoh_session.json5 /etc/zenoh/session.json5`). The router uses defaults and listens on `tcp/[::]:7447`.

This eliminates DDS's multicast discovery problems on Docker Desktop (macOS, Windows) and reduces log noise considerably. `ROS_DOMAIN_ID=42` is still respected by Zenoh, so the stack remains isolated from other ROS 2 nodes on the host.

### File-server design note

ROS 1 demos rely on the custom [`ros_file_server`](https://github.com/gramaziokohler/ros_file_server) package, which exposes a ROS service (`/file_server/get_file`) so that `roslibpy` clients can fetch mesh files referenced by `package://...` URLs.

There is no equivalent in the ROS 2 ecosystem. Rather than porting that package, this demo serves the entire `/opt/ros/jazzy/share/` directory over HTTP. A `package://my_pkg/path/to/foo.stl` reference becomes `http://localhost:9091/my_pkg/path/to/foo.stl`. Use `compas_fab.backends.HttpFileServerLoader` on the client side.

The URDF itself is published by `robot_state_publisher` on the `/robot_description` topic and can be fetched through rosbridge — no file-server call is needed for the URDF, only for the meshes it references.

## Using a local X server instead of the web GUI

For a faster, native RViz window (XQuartz on macOS, VcXsrv on Windows, native on Linux):

1. In `docker-compose.yml`, comment `DISPLAY=gui:0.0` in `moveit-demo` and uncomment `DISPLAY=host.docker.internal:0.0`.
2. On macOS/Windows, allow connections from network clients. On Linux you may need `xhost +local:docker`.

You can then also drop the `gui` service from `moveit-demo.depends_on` and stop the `gui` container if you don't need the web view.

## Networking

All services share the default Docker network. Because the stack uses Zenoh with a dedicated router, discovery is unicast TCP only — there are no multicast issues on Docker Desktop (macOS, Windows). Each ROS 2 node opens a TCP session to `tcp/zenoh-router:7447` and the router handles the rest.
