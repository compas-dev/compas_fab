"""Calculates a motion path.
    Inputs:
        robot: The robot
        group: str, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        start_configuration: :class:`compas_fab.robots.Configuration`, optional
            The robot's full configuration, i.e. values for all configurable
            joints of the entire robot cell, at the starting position. Defaults
            to the all-zero configuration.
        goal_constraints: list of :class:`compas_fab.robots.Constraint`
            The goal to be achieved, defined in a set of constraints.
            Constraints can be very specific, for example defining value domains
            for each joint, such that the goal configuration is included,
            or defining a volume in space, to which a specific robot link (e.g.
            the end-effector) is required to move to.
        planner_id: str
            The name of the algorithm used for path planning. Defaults to 'RRT'.
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

guid = str(ghenv.Component.InstanceGuid)
response_key = "response_" + guid
if response_key not in sc.sticky:
    sc.sticky[response_key] = None


if robot and robot.client and start_configuration and compute:
    if robot.client.is_connected:
        sc.sticky[response_key] = robot.plan_motion(goal_constraints,
                                                    start_configuration,
                                                    group,
                                                    {'planner_id': str(planner_id),
                                                     'attached_collision_meshes': list(attached_collision_meshes)})
    else:
        print("Robot client is not connected.")

trajectory = sc.sticky[response_key]
