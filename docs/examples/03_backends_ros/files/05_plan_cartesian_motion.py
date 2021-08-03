import time

from compas.datastructures import Mesh
from compas.geometry import Frame
from compas.robots import Configuration

import compas_fab
from compas_fab.backends import RosClient
from compas_fab.robots import CollisionMesh, AttachedCollisionMesh

with RosClient() as client:
    robot = client.load_robot()
    assert robot.name == 'ur5'

    ee_link_name = robot.get_end_effector_link_name()
    mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
    cm = CollisionMesh(mesh, 'tip')
    acm = AttachedCollisionMesh(cm, link_name=ee_link_name)
    client.add_attached_collision_mesh(acm)

    frames = []
    frames.append(Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0]))
    frames.append(Frame([0.5, 0.1, 0.6], [1, 0, 0], [0, 1, 0]))
    start_configuration = Configuration.from_revolute_values([-0.042, 0.033, -2.174, 5.282, -1.528, 0.000])
    options = {
        'max_step': 0.01,
        'avoid_collisions': True,
    }

    trajectory = robot.plan_cartesian_motion(frames,
                                             start_configuration,
                                             options=options)

    print("Computed cartesian path with %d configurations, " % len(trajectory.points))
    print("following %d%% of requested trajectory." % (trajectory.fraction * 100))
    print("Executing this path at full speed would take approx. %.3f seconds." % trajectory.time_from_start)
    print("Path plan computed with the attached collision meshes: {}".format(
        [acm.collision_mesh.id for acm in trajectory.attached_collision_meshes]))
