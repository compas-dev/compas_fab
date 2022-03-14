import time

from compas.datastructures import Mesh
from compas.geometry import Frame

import compas_fab
from compas_fab.backends import RosClient
from compas_fab.robots import PlanningScene
from compas_fab.robots import Tool

with RosClient() as client:
    robot = client.load_robot()
    scene = PlanningScene(robot)
    assert robot.name == 'ur5_robot'

    # create collision object
    mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
    t1cf = Frame([0.14, 0, 0], [0, 0, 1], [0, 1, 0])          # TODO: check this frame!
    tool = Tool(mesh, t1cf, name='tip')
    scene.add_attached_tool(tool)

    # sleep a bit before removing the tip
    time.sleep(1)

    # check if it's really there
    planning_scene = robot.client.get_planning_scene()
    acm = planning_scene.robot_state.attached_collision_objects
    assert acm[0].object['id'].startswith('tip_')

    scene.remove_attached_tool()

    planning_scene = robot.client.get_planning_scene()
    assert acm[0].object['id'] not in [c.object['id'] for c in planning_scene.robot_state.attached_collision_objects]
