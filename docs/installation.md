# Installation

This page covers installing `compas_fab` and setting up the front-ends
(CAD environments) and back-ends (planners) you intend to use.

We recommend [uv](https://docs.astral.sh/uv/) for managing your Python
environment, but `pip` and `conda` work too.

## Install uv

If you do not have `uv` installed, follow the instructions on their website or run:

=== "Mac/Linux"

    ```bash
    curl -LsSf https://astral.sh/uv/install.sh | sh
    ```

=== "Windows"

    ```powershell
    powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
    ```

## Create a virtual environment

```bash
uv venv
```

Activate it:

=== "Mac/Linux"

    ```bash
    source .venv/bin/activate
    ```

=== "Windows"

    ```powershell
    .venv\Scripts\activate
    ```

## Install compas_fab

```bash
uv pip install compas_fab
```

Or with `pip`:

```bash
pip install compas_fab
```

Or with `conda` from the `conda-forge` channel:

```bash
conda install -c conda-forge compas_fab
```

## Verify installation

```bash
python -m compas_fab
```

You should see a confirmation message that `compas_fab` is installed correctly.

## Front-ends (CAD environments)

`compas_fab` is CAD-independent. The core API works in any Python environment;
host bindings are provided for the following:

=== "Rhino 8"

    `compas_fab` is compatible with Rhino 8 and later. Add the requirement
    header to the top of your script in the Rhino 8 Script Editor:

    ```python
    # r: compas_fab
    ```

=== "Grasshopper"

    The Grasshopper components are bundled with `compas_fab`. After installing
    in a Rhino 8-compatible environment, the components are available in
    the Grasshopper toolbar.

=== "Blender"

    Install `compas_fab` into Blender's Python environment. See the
    [COMPAS Blender installation guide](https://compas.dev/compas/latest/userguide/cad.blender.html)
    for setting up Blender's Python first, then run `pip install compas_fab`
    inside that interpreter.

=== "COMPAS Viewer"

    The COMPAS viewer integration is available out of the box once
    `compas_viewer` is installed in the same environment:

    ```bash
    uv pip install compas_viewer
    ```

=== "VS Code (no CAD)"

    `compas_fab` is fully usable from a plain Python script without any CAD
    host — useful for headless planning, batch jobs, and testing.

## Back-ends (planners)

Pick the planning back-end that matches your needs. See the
[Backends](backends/index.md) section for detailed usage.

| Backend | Best for | Setup |
|---|---|---|
| **No backend** | Modeling, kinematics, visualization | None — works out of the box |
| **Analytical** | Closed-form IK for known robots (UR, Staubli) | None — built in |
| **PyBullet** | Fast IK and collision checking from a Python process | `uv pip install pybullet` (see notes) |
| **ROS / MoveIt 1** | Full motion planning with the ROS 1 ecosystem | Docker (legacy) |
| **ROS 2 / MoveIt 2** | Full motion planning with the ROS 2 ecosystem | Docker (recommended) |

### PyBullet installation note

PyBullet has no pre-built wheels for recent Python versions on some platforms
and may need to be built from source. On macOS:

```bash
CFLAGS="-fno-define-target-os-macros" uv pip install pybullet
```

### ROS / MoveIt back-ends

The ROS-based back-ends run inside Docker. See:

- [ROS / MoveIt 1](backends/ros.md) — legacy ROS 1 stack
- [ROS 2 / MoveIt 2](backends/ros2.md) — current ROS 2 stack with Zenoh transport

Both back-ends communicate with `compas_fab` over `rosbridge` (WebSocket).
You do not need a Linux machine; Docker Desktop on Mac or Windows is enough.
