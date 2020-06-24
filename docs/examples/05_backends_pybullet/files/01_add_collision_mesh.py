import time
from compas.datastructures import Mesh

import compas_fab
from compas_fab.backends.pybullet import PyBulletClient
from compas_fab.robots import CollisionMesh

with PyBulletClient() as client:
    urdf_filepath = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
    client.load_robot_from_urdf(urdf_filepath)

    mesh = Mesh.from_stl(compas_fab.get('planning_scene/floor.stl'))
    cm = CollisionMesh(mesh, 'floor')
    client.add_collision_mesh(cm)

    time.sleep(1)

    client.remove_collision_mesh('floor')

    time.sleep(1)
