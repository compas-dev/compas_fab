from compas.datastructures import Mesh

import compas_fab
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import RobotCell
from compas_fab.robots import RigidBody

with PyBulletClient() as client:

    # Create a robot cell
    robot_cell = RobotCell()  # Typically RobotCell is initialed with a Robot, but here it is empty for simplicity

    # Add some RigidBodies as stationary obstacles
    floor_mesh = Mesh.from_stl(compas_fab.get("planning_scene/floor.stl"))
    robot_cell.rigid_body_models["floor"] = RigidBody(floor_mesh)
    cone = Mesh.from_stl(compas_fab.get("planning_scene/cone.stl"))
    robot_cell.rigid_body_models["cone"] = RigidBody(cone)

    target_marker = Mesh.from_obj(compas_fab.get("planning_scene/target_marker.obj"))
    robot_cell.rigid_body_models["target_marker"] = RigidBody(target_marker)

    # The planner object is needed to pass the robot cell into the PyBullet client
    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)

    # No RobotCellState is passed in this example, so the rigid bodies are added to the PyBullet world's origin
    # The floor and cone should appear in the PyBullet's GUI
    input("Press Enter to continue...")
