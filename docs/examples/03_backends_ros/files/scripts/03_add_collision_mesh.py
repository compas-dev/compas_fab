import time

from compas.datastructures import Mesh

import compas_fab
from compas_fab.backends import RosClient
from compas_fab.robots import CollisionMesh
from compas_fab.robots import PlanningScene
from compas_fab.robots.ur5 import Robot

with RosClient() as client:
    robot = Robot(client)

    scene = PlanningScene(robot)
    mesh = Mesh.from_stl(compas_fab.get('planning_scene/floor.stl'))
    cm = CollisionMesh(mesh, 'floor')
    scene.add_collision_mesh(cm)

    # sleep a bit before terminating the client
    time.sleep(1)
