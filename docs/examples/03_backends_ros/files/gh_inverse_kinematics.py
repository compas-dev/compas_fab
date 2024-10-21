"""Calculate the robot's inverse kinematic for a given target.
    Inputs:
        planner: PlannerInterface
            A planning backend with inverse kinematics capability.
        target: The target to calculate the inverse kinematics for.
        robot_cell_state: The robot cell state
        group: str, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        start_configuration: :class:`compas_fab.robots.Configuration`, optional
            If passed, the inverse will be calculated such that the calculated
            joint positions differ the least from the start_configuration.
            Defaults to the zero configuration.
        avoid_collisions: bool, optional
            Whether or not to avoid collisions. Defaults to True.
    Output:
        group_configuration: The group configuration
        full_configuration: The full configuration
"""

from __future__ import print_function

if planner:
    robot_cell = planner.robot_cell
    if robot_cell and planner.client.is_connected:
        options = {"avoid_collisions": avoid_collisions}
        group_configuration = planner.inverse_kinematics(target, robot_cell_state, group=group, options=options)
        full_configuration = robot_cell.zero_full_configuration().merged(group_configuration)
        print(group_configuration)
    else:
        print("planner.client is not connected")
