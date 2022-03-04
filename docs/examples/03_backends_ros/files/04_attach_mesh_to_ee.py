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

    # create collision object
    mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
    cm = CollisionMesh(mesh, 'tip')

    # attach it to the end-effector
    group = robot.main_group_name
    scene.attach_collision_mesh_to_robot_end_effector(cm, group=group)

    # sleep a bit before removing the tip
    time.sleep(1)

    # check if it's really there
    planning_scene = robot.client.get_planning_scene()
    assert 'tip' in [c.object['id'] for c in planning_scene.robot_state.attached_collision_objects]

    scene.reset()

    planning_scene = robot.client.get_planning_scene()
    assert 'tip' not in [c.object['id'] for c in planning_scene.robot_state.attached_collision_objects]
