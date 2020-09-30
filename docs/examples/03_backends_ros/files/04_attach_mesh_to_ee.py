import time

import compas
from compas.datastructures import Mesh

from compas_fab.backends import RosClient
from compas_fab.robots import CollisionMesh
from compas_fab.robots import PlanningScene

with RosClient() as client:
    robot = client.load_robot()
    scene = PlanningScene(robot)
    assert robot.name == 'ur5'

    # create collision object
    mesh = Mesh.from_stl(compas.get('cone.stl'))
    cm = CollisionMesh(mesh, 'tip')

    # attach it to the end-effector
    group = robot.main_group_name
    scene.attach_collision_mesh_to_robot_end_effector(cm, group=group)

    # sleep a bit before removing the tip
    time.sleep(1)

    scene.remove_attached_collision_mesh('tip')
