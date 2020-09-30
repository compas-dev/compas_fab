import time

import compas
import compas_fab
from compas_fab.backends.pybullet import PyBulletClient
from compas_fab.robots import AttachedCollisionMesh
from compas_fab.robots import CollisionMesh

from compas.datastructures import Mesh

with PyBulletClient() as client:
    urdf_filepath = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
    robot = client.load_robot(urdf_filepath)

    mesh = Mesh.from_stl(compas.get('cone.stl'))
    cm = CollisionMesh(mesh, 'tip')
    acm = AttachedCollisionMesh(cm, 'ee_link')
    client.add_attached_collision_mesh(acm, {'mass': 1, 'robot': robot})

    time.sleep(1)
    client.step_simulation()
    time.sleep(1)

    client.remove_attached_collision_mesh('tip')

    time.sleep(1)
