# Core

Platform-independent modules. These work everywhere `compas_fab` can be
imported and form the foundation that the integrations build on top of.

- **[compas_fab.robots](../compas_fab.robots.md)**: robot cells, tools,
  rigid bodies, targets, waypoints, and the robot library.
- **[compas_fab.backends](../compas_fab.backends.md)**: backend client
  and planner classes (analytical, PyBullet, ROS/MoveIt). Each planner
  exposes its full set of FK/IK/motion-planning methods directly; the
  building blocks behind them (interfaces and backend features) are covered
  by [Backend architecture](../../developer/architecture.md) in the developer
  guide.
- **[compas_fab.utilities](../compas_fab.utilities.md)**: assorted
  helpers (file I/O, geometry, math).
