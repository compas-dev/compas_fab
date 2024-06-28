from compas.datastructures import Mesh

import compas_fab
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import RobotCell
from compas_fab.robots import RigidBody

with PyBulletClient() as client:

    # Create a robot cell and add objects to it
    robot_cell = RobotCell()  # Typically RobotCell is initialed with a RobotModel, but here it is empty for simplicity
    floor_mesh = Mesh.from_stl(compas_fab.get("planning_scene/floor.stl"))
    robot_cell.rigid_body_models["floor"] = RigidBody(floor_mesh)
    cone = Mesh.from_stl(compas_fab.get("planning_scene/cone.stl"))
    robot_cell.rigid_body_models["cone"] = RigidBody(cone)

    # The planner is used for passing the robot cell into the PyBullet client
    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)
    # The floor should appear in the PyBullet's GUI

    input("Press Enter to continue...")
