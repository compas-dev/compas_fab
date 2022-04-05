import os
import math
from compas.geometry import Point
from compas.geometry import Plane
from compas.geometry import Frame
from compas.geometry import Sphere

from compas_fab.backends import AnalyticalInverseKinematics
from compas_fab.backends import PyBulletClient
from compas_fab.robots import ReachabilityMap
from compas_fab.robots.reachability_map import DeviationVectorsGenerator
from compas_fab import DATA


# 1. Define generators

sphere = Sphere((0.4, 0, 0), 0.15)


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


def deviation_vector_generator(frame):
    for xaxis in DeviationVectorsGenerator(frame.xaxis, math.radians(40), 1):
        yaxis = frame.zaxis.cross(xaxis)
        yield Frame(frame.point, xaxis, yaxis)

# 2. Create 2D generator


def generator():
    for frame in points_on_sphere_generator(sphere):
        yield deviation_vector_generator(frame)

# 3. Create reachability map 2D


with PyBulletClient(connection_type='direct') as client:
    # load robot and define settings
    robot = client.load_ur5(load_geometry=True)
    ik = AnalyticalInverseKinematics(client)
    client.inverse_kinematics = ik.inverse_kinematics
    options = {"solver": "ur5", "check_collision": True, "keep_order": True}
    # calculate reachability map
    map = ReachabilityMap()
    map.calculate(generator(), robot, options)
    # save to json
    map.to_json(os.path.join(DATA, "reachability", "map2D_deviation.json"))
