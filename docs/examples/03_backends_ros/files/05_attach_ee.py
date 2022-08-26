import time

from compas.datastructures import Mesh
from compas.geometry import Frame

import compas_fab
from compas_fab.backends import RosClient
from compas_fab.robots import Tool

with RosClient() as client:
    robot = client.load_robot()
    assert robot.name == 'ur5_robot'

    # create collision object
    mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
    t1cf = Frame([0.14, 0, 0], [0, 0, 1], [0, 1, 0])
    tool = Tool(mesh, t1cf, name='tip')
    robot.attach_tool(tool)
