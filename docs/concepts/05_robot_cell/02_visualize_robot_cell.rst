.. _visualize_robot_cell:

********************************************************************************
Robot Cell Visualization
********************************************************************************

A RobotCell object must be visualized with a matching RobotCellState object.
The RobotCell contains all the geometrical and kinematic models of the robot,
tools and rigid bodies, while the RobotCellState describes the configuration
and pose of these objects.

A CAD-specific SceneObject class is available for visualizing the robot cell in
different CAD environments, such as Rhino, Grasshopper, Compas Viewer, etc.
The SceneObject is created from a RobotCell, after which it is responsible for
drawing the 3D geometry in the CAD environment.

These SceneObject instances designed such that the time-consuming process of
converting compas geometry to CAD-specific geometry is done only once.
Once the SceneObject is initialized, it can be used to visualize different
RobotCellStates quickly.
However, the SceneObject instances must be recreated whenever the RobotCell changes.

Initial Draw
======================


Update State
======================

