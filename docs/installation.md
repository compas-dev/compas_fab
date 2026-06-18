# Installation

Install the `compas_fab` library itself. For the CAD environment you plan to
use, also see [CAD front-ends](frontends.md). For the planning backend you
plan to use, see [Choosing a backend](backends/index.md).

We recommend [uv](https://docs.astral.sh/uv/) for managing your Python
environment, but `pip` and `conda` work too.

## Install uv

If you do not have `uv` installed:

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

## Next steps

- **[Choose a backend](backends/index.md)**: pick a planner that matches what
  you want to do.
- **[Set up a CAD front-end](frontends.md)**: Rhino, Grasshopper, Blender,
  COMPAS Viewer or headless Python.
- **[Concepts](concepts.md)**: a backend-agnostic walkthrough of the data
  model (`RobotCell`, `RobotCellState`, `Target`, `Waypoints`).
