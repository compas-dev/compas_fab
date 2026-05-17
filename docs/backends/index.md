# Backends

`compas_fab` works with several planning back-ends. Pick the one that matches
your needs:

| Backend | Connection | Capabilities |
|---|---|---|
| **No backend** | Local | Modeling, kinematics, visualization |
| **[Analytical](analytical.md)** | Local | Closed-form IK for known robot families |
| **[PyBullet](pybullet.md)** | Local | Fast IK + collision checking |
| **[ROS / MoveIt 1](ros.md)** | Docker via rosbridge | Full motion planning (legacy) |
| **[ROS 2 / MoveIt 2](ros2.md)** | Docker via rosbridge | Full motion planning (current) |

All ROS-based back-ends communicate with `compas_fab` over `rosbridge`
WebSockets, so they are usable from any operating system that runs Docker.

## Without a back-end

Even without a planning back-end, the core of `compas_fab` is useful for:

- Creating and loading `RobotCell` instances with tools and rigid bodies
- Modeling cell state via `RobotCellState`
- Authoring targets and waypoints, and serializing them for planning elsewhere
- Visualizing in a CAD front-end
- Forward kinematics via `compas_robots.RobotModel.forward_kinematics`
- Analytical inverse kinematics via the built-in analytical backend
  (no collision checking)
