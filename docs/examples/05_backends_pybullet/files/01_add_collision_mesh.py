import time
from compas.datastructures import Mesh

import compas_fab
from compas_fab.backends import PyBulletClient
from compas_fab.robots import CollisionMesh

with PyBulletClient() as client:
    urdf_filepath = compas_fab.get('robot_library/ur5_robot/urdf/robot_description.urdf')
    robot = client.load_robot(urdf_filepath)

    mesh = Mesh.from_stl(compas_fab.get('planning_scene/floor.stl'))
    cm = CollisionMesh(mesh, 'floor')
    client.add_collision_mesh(cm)

    time.sleep(1)

    client.remove_collision_mesh('floor')

    time.sleep(1)
