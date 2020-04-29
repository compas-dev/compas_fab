"""Calculates a cartesian motion path (linear in tool space).
    Inputs:
        robot: The robot
        group: str, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        planes: The planes in world coordinate frame through which the path is defined.
        start_configuration: :class:`compas_fab.robots.Configuration`, optional
            The robot's full configuration, i.e. values for all configurable
            joints of the entire robot cell, at the starting position. Defaults
            to the all-zero configuration.
        max_step: float
            The approximate distance between the calculated points. (Defined in
            the robot's units)
        avoid_collisions: bool, optional
            Whether or not to avoid collisions. Defaults to True.
        attached_collision_meshes: list of :class:`compas_fab.robots.AttachedCollisionMesh`
            Defaults to None.
        compute: bool
            Press to calculate solution.
    Output:
        :class:`compas_fab.robots.JointTrajectory`
            The calculated trajectory.
"""
from __future__ import print_function
import scriptcontext as sc

from compas.geometry import Frame


guid = str(ghenv.Component.InstanceGuid)
response_key = "response_" + guid
if response_key not in sc.sticky:
    sc.sticky[response_key] = None

frames = [Frame(plane.Origin, plane.XAxis, plane.YAxis) for plane in planes]

if robot and robot.client and start_configuration and compute:
    if robot.client.is_connected:
        options = {
            'max_step': float(max_step),
            'avoid_collisions': bool(avoid_collisions),
            'attached_collision_meshes': list(attached_colllision_meshes),
        }
        sc.sticky[response_key] = robot.plan_cartesian_motion(frames,
                                                              start_configuration=start_configuration,
                                                              group=group,
                                                              options=options)
    else:
        print("Robot client is not connected")

trajectory = sc.sticky[response_key]
