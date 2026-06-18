import math
import os

from compas.geometry import Frame
from compas.geometry import Plane
from compas.geometry import Point
from compas.geometry import Sphere
from compas.geometry import Vector

from compas_fab import DATA
from compas_fab.backends import AnalyticalPyBulletClient
from compas_fab.backends import AnalyticalPyBulletPlanner
from compas_fab.backends import UR5Kinematics
from compas_fab.robots import ReachabilityMap
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode

# 1. Define generators


def points_on_sphere_generator(sphere):
    for theta_deg in range(0, 360, 20):
        for phi_deg in range(0, 90, 10):
            theta = math.radians(theta_deg)
            phi = math.radians(phi_deg)
            x = sphere.point.x + sphere.radius * math.cos(theta) * math.sin(phi)
            y = sphere.point.y + sphere.radius * math.sin(theta) * math.sin(phi)
            z = sphere.point.z + sphere.radius * math.cos(phi)
            point = Point(x, y, z)
            axis = sphere.point - point
            plane = Plane((x, y, z), axis)
            f = Frame.from_plane(plane)
            # for UR5 is zaxis the xaxis
            yield Frame(f.point, f.zaxis, f.yaxis)


def sphere_generator():
    sphere = Sphere((0.35, 0, 0), 0.15)
    for x in range(5):
        for z in range(7):
            center = sphere.point + Vector(x, 0, z) * 0.05
            yield Sphere(center, sphere.radius)


# 2. Create 2D generator


def generator():
    for sphere in sphere_generator():
        yield points_on_sphere_generator(sphere)


# 3. Create reachability map 2D

with AnalyticalPyBulletClient(connection_type="direct") as client:
    # load robot and define settings
    robot_cell, robot_cell_state = RobotCellLibrary.ur5()
    planner = AnalyticalPyBulletPlanner(client, UR5Kinematics())
    planner.set_robot_cell(robot_cell, robot_cell_state)

    options = {"solver": "ur5", "check_collision": True, "keep_order": True}
    # calculate reachability map
    map = ReachabilityMap()
    map.calculate(generator(), planner, robot_cell_state, TargetMode.ROBOT, options)
    # save to json
    map.to_json(os.path.join(DATA, "reachability", "map2D_spheres.json"))
