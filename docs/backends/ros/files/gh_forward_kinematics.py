"""Calculate the robot's forward kinematic.
    Inputs:
        robot: :class:`compas_fab.robots.Robot`
            The robot.
        group: str, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        full_configuration : :class:`compas_fab.robots.Configuration`
            The full configuration to calculate the forward kinematic for. If no
            full configuration is passed, the zero-joint state for the other
            configurable joints is assumed.
    Output:
        :class:`Frame`
            The frame in the world coordinate frame (WCF).
"""
from __future__ import print_function

if robot and robot.client and full_configuration:
    if robot.client.is_connected:
        frame_WCF = robot.forward_kinematics(full_configuration, group, options={'solver': 'model'})
        print(frame_WCF)
    else:
        print("Robot is not connected")
