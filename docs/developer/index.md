# Extending compas_fab

This section is for contributors extending `compas_fab` — adding a backend,
building the Grasshopper components, or working on the internals.

- **[Backend architecture](architecture.md)**: how clients, planners and
  backend features fit together; the contract a new backend must implement.
  The API reference for the extension points lives alongside it:
  [interfaces](../api/compas_fab.backends.interfaces.md) (the abstract
  contracts) and the per-backend implementations for
  [ROS / MoveIt](../api/compas_fab.backends.ros.backend_features.md),
  [PyBullet](../api/compas_fab.backends.pybullet.backend_features.md) and
  [analytical kinematics](../api/compas_fab.backends.kinematics.backend_features.md).
- **[ActionChain design notes](action.md)**: technical notes regarding the current and future implementation of the `Action` / `ActionChain` data containers.
- **[Grasshopper components](grasshopper.md)**: building and installing
  the Grasshopper user objects.
- **[Icon system](icons/index.md)**: details of the design system used for the Grasshopper icons.
