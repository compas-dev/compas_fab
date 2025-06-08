.. _gh_modify_configuration_interactively:

==============================================================
Plan Motion and Visualize Trajectory
==============================================================

The following example shows how to interactively modify the robot configuration
and visualize it in Grasshopper.


Plan Motion
===========

.. code-block:: python

    # venv: compas_fab
    from compas_fab.robots import ConfigurationTarget

    # Create Configuration Target
    configuration_target = ConfigurationTarget(target_configuration)

    # Plan motion to the target configuration
    trajectory = planner.plan_motion(configuration_target, start_state)

Convert Trajectory to Configuration to Robot Cell State
==============================================================

The returned trajectory from the planner is a list of points,
the following code allows you to retrieve one of the points
and merge it with the robot cell state for visualization.

.. code-block:: python

    from compas_robots import Configuration

    # Convert fraction number to index of trajectory point
    config_index = round(fraction * (len(trajectory.points)-1))
    print(config_index)

    # The start_configuration of the trajectory has names and joint type info
    configuration = trajectory.start_configuration.copy()
    configuration.joint_values =  trajectory.points[config_index].joint_values

    # Merge configuration with a copy of the state
    robot_cell_state_out = start_state.copy()
    robot_cell_state_out.robot_configuration.merge(configuration)