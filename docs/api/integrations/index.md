# Integrations

Host-specific bindings for CAD environments. Each module is only importable
when its host is available; pick the one that matches where your code runs.

- **[compas_fab.blender](../compas_fab.blender.md)**: Blender scene objects.
- **[compas_fab.ghpython](../compas_fab.ghpython.md)**: Grasshopper components.
- **[compas_fab.rhino](../compas_fab.rhino.md)**: Rhino scene objects.

For the COMPAS viewer, robot scene objects come from
[compas_robots.viewer](https://compas.dev/compas_robots/latest/api/compas_robots.viewer/);
`compas_fab` itself does not ship a separate viewer integration.
