# COMPAS FAB

**Robotic fabrication package for the COMPAS Framework**, facilitating the
planning and execution of robotic fabrication processes. It provides interfaces
to existing software libraries and tools available in the field of robotics
(e.g. OMPL, ROS, MoveIt) and makes them accessible from within the parametric
design environment. The package builds on top of
[COMPAS](https://compas.dev/) and
[COMPAS Robots](https://compas.dev/compas_robots/latest/).

## Main features

- Built on top of standard model description formats (URDF, SRDF)
- Motion planning with multiple backends (ROS/MoveIt, PyBullet, analytical)
- Execution of planned trajectories on real or simulated robots
- CAD-independent core, with bindings for Rhino, Grasshopper, Blender and the COMPAS viewer
- Robotic fabrication process modeling

```pycon
>>> from compas_fab.robots import RobotCellLibrary
>>> cell, state = RobotCellLibrary.ur10e()
>>> print(cell.robot_model)
Robot name=ur10e, Links=11, Joints=10 (6 configurable)
```

## Where to go next

- **[Installation](installation.md)**: set up `compas_fab` with the front-end
  and back-end of your choice.
- **[Tutorial](tutorial.md)**: a tour of the main concepts and APIs.
- **[Backends](backends/index.md)**: how to use ROS/MoveIt, ROS 2/MoveIt 2,
  PyBullet, and the analytical kinematics backends.
- **[API Reference](api/index.md)**: the full module-level documentation.
