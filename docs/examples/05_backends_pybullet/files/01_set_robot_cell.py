from compas.datastructures import Mesh

import compas_fab
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import RobotCell
from compas_fab.robots import RigidBody

with PyBulletClient() as client:
    urdf_filepath = compas_fab.get("robot_library/ur5_robot/urdf/robot_description.urdf")
    robot = client.load_robot(urdf_filepath)

    robot_cell = RobotCell(robot)
    floor_mesh = Mesh.from_stl(compas_fab.get("planning_scene/floor.stl"))
    robot_cell.rigid_body_models["floor"] = RigidBody(floor_mesh)

    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)
    # The floor should appear in the PyBullet's GUI

    input("Press Enter to continue...")
