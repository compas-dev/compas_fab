import os
import math
from compas.geometry import Point
from compas.geometry import Plane
from compas.geometry import Frame
from compas.geometry import Sphere

from compas_fab.backends import AnalyticalInverseKinematics
from compas_fab.backends import PyBulletClient
from compas_fab.robots import ReachabilityMap
from compas_fab import DATA

# 1. Define frames on a sphere
sphere = Sphere((0.4, 0, 0), 0.15)

frames = []
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
        frames.append(Frame(f.point, f.zaxis, f.yaxis))

# 2. Create ReachabilityMap 1D

with PyBulletClient(connection_type='direct') as client:

    robot = client.load_ur5(load_geometry=True)
    ik = AnalyticalInverseKinematics(client)
    # set a new IK function
    client.inverse_kinematics = ik.inverse_kinematics
    options = {"solver": "ur5", "check_collision": True, "keep_order": True}

    map = ReachabilityMap()
    map.calculate(frames, robot, options)
    map.to_json(os.path.join(DATA, "reachability", "map1D.json"))
