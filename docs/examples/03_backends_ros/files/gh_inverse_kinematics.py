"""Calculate the robot's inverse kinematic for a given plane.
    Inputs:
        planner: PlannerInterface
            A planning backend with inverse kinematics capability.
        group: str, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        plane: The plane in world coordinate frame to calculate the inverse for.
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

from compas.geometry import Frame

frame = Frame(plane.Origin, plane.XAxis, plane.YAxis)

if planner:
    robot = planner.robot
    if planner.client.is_connected:
        options = {"avoid_collisions": avoid_collisions, "return_full_configuration": True}
        full_configuration = planner.inverse_kinematics(
            frame, start_configuration, group=group, return_full_configuration=True, options=options
        )
        group_configuration = robot.get_group_configuration(group, full_configuration)
        print(group_configuration)
    else:
        print("planner.client is not connected")
