"""Visualizes a robot.
    Inputs:
        robot: The robot
        group: str
            The planning group used for visualisation
        full_configuration: :class:`compas_fab.robots.Configuration`
            The robot's full configuration to display.
        show_visual: bool
           Whether or not to show the robot's visual meshes.
        show_collision: bool
           Whether or not to show the robot's collision meshes.
        show_frames: bool
           Whether or not to show the robot's joint frames.
        show_base_frame: bool
           Whether or not to show the robot's base frame.
        show_end_effector_frame: bool
            Whether or not to show the robot's end-effector frame.
    Output:
        visual: The robot's visual meshes.
        collision: The robot's collision meshes.
        frames: The robot's joint frames.
        base_frame_WCF: The robot's base frame.
        ee_frame_WCF: The robot's end-effector frame.
"""
from __future__ import print_function

if robot and full_configuration:
    robot.update(full_configuration, visual=show_visual, collision=show_collision)
    print(full_configuration)

    if show_visual:
        visual = robot.draw_visual()

    if show_collision:
        collision = robot.draw_collision()

    if show_frames:
        axes = robot.transformed_axes(full_configuration)
        frames = robot.transformed_frames(full_configuration)

    if show_base_frame:
        base_frame_WCF = robot._get_current_base_frame(full_configuration, group)

    if show_end_effector_frame:
        ee_frame_RCF = robot.forward_kinematics(full_configuration, group=group, backend='model')
        ee_frame_WCF = robot.to_world_coords(ee_frame_RCF, group=group)
        print("End-effector", ee_frame_WCF)
