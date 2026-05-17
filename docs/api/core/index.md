# Core

Platform-independent modules. These work everywhere `compas_fab` can be
imported and form the foundation that the integrations build on top of.

- **[compas_fab](../compas_fab.md)**: top-level package exports.
- **[compas_fab.robots](../compas_fab.robots.md)**: robot cells, tools,
  rigid bodies, targets, waypoints, and the robot library.
- **[compas_fab.backends](../compas_fab.backends.md)**: backend client
  and planner classes (analytical, PyBullet, ROS/MoveIt).
- **[compas_fab.backends.ros](../compas_fab.backends.ros.md)**: the ROS
  rosbridge client, MoveIt planner, and ROS-specific message helpers.
- **[compas_fab.backends.pybullet](../compas_fab.backends.pybullet.md)**:
  the PyBullet client and planner.
- **[compas_fab.utilities](../compas_fab.utilities.md)**: assorted
  helpers (file I/O, geometry, math).
