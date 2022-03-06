import time

from compas.datastructures import Mesh

import compas_fab
from compas_fab.backends import RosClient
from compas_fab.robots import CollisionMesh
from compas_fab.robots import PlanningScene

with RosClient() as client:
    robot = client.load_robot()
    scene = PlanningScene(robot)
    assert robot.name == 'ur5_robot'

    mesh = Mesh.from_stl(compas_fab.get('planning_scene/floor.stl'))
    cm = CollisionMesh(mesh, 'floor')
    scene.add_collision_mesh(cm)

    # sleep a bit before removing the floor
    time.sleep(1)

    scene.remove_collision_mesh('floor')
