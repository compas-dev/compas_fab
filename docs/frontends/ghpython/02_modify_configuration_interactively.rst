.. _gh_modify_configuration_interactively:

==============================================================
Interactively modify Robot Configuration and Visualization
==============================================================

The following example shows how to visualize a robot from the COMPAS FAB library in Grasshopper.
All the code is placed in a single Python Script component for simplicity.

.. code-block:: python

    # r: compas_fab
    # venv: compas_fab

    from compas_fab.robots import RobotCellLibrary
    from compas.scene import Scene

    # Load the robot model
    robot_cell, robot_cell_state = RobotCellLibrary.ur5()

    # Create a scene object for visualization
    scene = Scene()
    scene_object = scene.add(robot_cell)

    # Visualize the robot in the COMPAS Viewer or other CAD environment
    native_geometry = scene_object.draw(robot_cell_state)