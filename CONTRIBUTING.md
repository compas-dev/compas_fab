# Contributor's Guide

Contributions are welcome and greatly appreciated.

## Code contributions

1. Fork the [COMPAS FAB repository](https://github.com/compas-dev/compas_fab)
   and clone your fork.
2. Create and activate a virtual environment using `venv`, `uv`, `conda`, or
   another environment manager.
3. From the repository root, install the package in editable mode together
   with its development tools and optional test backends:

   ```bash
   python -m pip install -e ".[dev,pybullet,pyroki]"
   ```

   PyRoKI requires Python 3.10 or newer. Python 3.12 is the version used by the
   integration workflow. If your change does not touch an optional backend,
   `python -m pip install -e ".[dev]"` is sufficient for the core suite.

4. Run the ordinary test suite:

   ```bash
   pytest
   ```

5. For ROS integration work, start both test stacks:

   ```bash
   docker compose -f tests/integration_setup/docker-compose.yml up -d --build
   docker compose -f tests/integration_setup/docker-compose-ros2.yml up -d --build
   ```

   Then opt into the live ROS tests and module doctests:

   ```bash
   COMPAS_FAB_RUN_ROS_INTEGRATION_TESTS=1 pytest --doctest-modules
   ```

   In PowerShell, set the environment variable first:

   ```powershell
   $env:COMPAS_FAB_RUN_ROS_INTEGRATION_TESTS = "1"
   pytest --doctest-modules
   ```

6. Run the style checks:

   ```bash
   invoke lint
   ```

7. Add a changelog entry for user-visible behavior. Add yourself to
   `AUTHORS.md` if this is your first contribution.
8. Commit your changes, push your branch, and open a pull request against
   `main`.

When you finish integration testing, stop both stacks:

```bash
docker compose -f tests/integration_setup/docker-compose.yml down
docker compose -f tests/integration_setup/docker-compose-ros2.yml down
```

## Development commands

The repository uses [Invoke](https://www.pyinvoke.org/) for recurring tasks:

- `invoke clean`: remove generated artifacts.
- `invoke check`: check documentation and code consistency.
- `invoke docs`: build the MkDocs site.
- `invoke lint`: run code-style checks.
- `invoke test`: run the test suite.
- `invoke testdocs`: test examples in docstrings.
- `invoke testcodeblocks`: test examples in documentation code blocks.
- `invoke`: list all available tasks.

## Documentation improvements

Documentation source is Markdown under `docs/` and is built with
[MkDocs](https://www.mkdocs.org/). API pages use MkDocstrings and NumPy-style
docstrings. See the [NumPy docstring standard](https://numpydoc.readthedocs.io/en/latest/format.html)
for examples.

Build the site locally after changing documentation:

```bash
invoke docs
```

The same guide is included in the MkDocs site through
`docs/developer/contributing.md`; keep the root file as the canonical source.

## Bug reports

When [reporting a bug](https://github.com/compas-dev/compas_fab/issues), include:

- Operating-system name and version.
- Python version.
- Relevant environment and dependency details.
- Minimal, reproducible steps and the complete error message.

## Feature requests and feedback

Use the [issue tracker](https://github.com/compas-dev/compas_fab/issues) for
feature proposals and feedback. Explain the use case, describe the proposed
behavior, and keep the initial scope as focused as practical.
