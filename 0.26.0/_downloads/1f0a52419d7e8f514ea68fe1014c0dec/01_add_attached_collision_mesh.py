import time

from compas.datastructures import Mesh
from compas.robots import LocalPackageMeshLoader

import compas_fab
from compas_fab.backends import PyBulletClient
from compas_fab.robots import AttachedCollisionMesh
from compas_fab.robots import CollisionMesh


with PyBulletClient() as client:
    urdf_filepath = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
    loader = LocalPackageMeshLoader(compas_fab.get('universal_robot'), 'ur_description')
    robot = client.load_robot(urdf_filepath, [loader])

    mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
    cm = CollisionMesh(mesh, 'tip')
    acm = AttachedCollisionMesh(cm, 'ee_link')
    client.add_attached_collision_mesh(acm, {'mass': 0.5, 'robot': robot})

    time.sleep(1)
    client.step_simulation()
    time.sleep(1)

    client.remove_attached_collision_mesh('tip', {'robot': robot})

    time.sleep(1)
