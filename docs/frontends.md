# CAD front-ends

**COMPAS FAB** is CAD-independent. The core API works in any Python
environment; host bindings are provided for several CAD environments and
viewers.

You only need to set up the front-end you actually use. The planning
backend ([choose one](backends/index.md)) is a separate decision.

## Rhino 8

`compas_fab` is compatible with Rhino 8 and later. Add the requirement
header to the top of your script in the Rhino 8 Script Editor:

```python
# r: compas_fab
```

Rhino 8 ships with CPython 3.9, which is why `compas_fab` keeps 3.9 as its
minimum Python version.

## Grasshopper

The Grasshopper components are installed via the Rhino Package Manager. Search for 
`compas_fab` in the Rhino Package Manager and install it. After installing it, the
components appear in the Grasshopper toolbar.

For developing the components, see
[Developer guide › Grasshopper components](developer/grasshopper.md).

## Blender

Install `compas_fab` into Blender's bundled Python. See the
[COMPAS Blender installation guide](https://compas.dev/compas/latest/userguide/cad.blender.html)
for setting up Blender's Python, then run `pip install compas_fab` inside
that interpreter.

## COMPAS Viewer

The COMPAS viewer integration is available out of the box once
`compas_viewer` is installed in the same environment:

```bash
uv pip install compas_viewer
```

Robot scene objects come from
[`compas_robots.viewer`](https://compas.dev/compas_robots/latest/api/compas_robots.viewer/).

## Headless / VS Code

`compas_fab` is fully usable from a plain Python script without any CAD
host, which is useful for headless planning, batch jobs, CI, and more.

## Which front-end works with which backend?

| Front-end | Analytical | Analytical + PyBullet | PyBullet | ROS |
|---|:-:|:-:|:-:|:-:|
| Rhino 8 | ✓ | — | — | ✓ (over WebSocket) |
| Grasshopper | ✓ | — | — | ✓ (over WebSocket) |
| Blender | ✓ | ✓ | ✓ | ✓ |
| COMPAS Viewer | ✓ | ✓ | ✓ | ✓ |
| Headless / VS Code | ✓ | ✓ | ✓ | ✓ |

PyBullet is notoriously difficult to run inside Rhino's
Python because PyBullet ships no wheels for newer Python
versions and doesn't compile easily.
