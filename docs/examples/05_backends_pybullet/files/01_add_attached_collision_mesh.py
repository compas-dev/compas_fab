import time

import compas_fab
from compas_fab.backends.pybullet import PyBulletClient
from compas_fab.robots import AttachedCollisionMesh, Configuration
from compas_fab.robots import CollisionMesh

from compas.datastructures import Mesh

with PyBulletClient() as client:
    urdf_filepath = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
    robot = client.load_robot(urdf_filepath)

    mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
    cm = CollisionMesh(mesh, 'tip')
    acm = AttachedCollisionMesh(cm, 'ee_link')
    client.add_attached_collision_mesh(acm, {'mass': 1, 'robot': robot})

    conf = Configuration.from_revolute_values([-1.0,0,0, 0,0,0])
    client.set_robot_configuration(robot, conf)

    count = 0
    while count < 30:
        time.sleep(.25)
        client.step_simulation()
        count += 1
    input('update conf!')

    client.remove_attached_collision_mesh('tip')
    input('Removed!')
