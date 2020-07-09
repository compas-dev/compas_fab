import time

import compas_fab
from compas_fab.backends.pybullet import PyBulletClient
from compas_fab.robots import AttachedCollisionMesh
from compas_fab.robots import CollisionMesh

from compas.datastructures import Mesh

with PyBulletClient() as client:
    urdf_filepath = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
    client.load_robot_from_urdf(urdf_filepath)

    mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
    cm = CollisionMesh(mesh, 'tip')
    acm = AttachedCollisionMesh(cm, 'ee_link')
    client.add_attached_collision_mesh(acm)

    time.sleep(1)
    client.step_simulation()
    time.sleep(1)

    client.remove_attached_collision_mesh('tip')

    time.sleep(1)
