from compas.datastructures import Mesh

import compas_fab
from compas_fab.backends import RosClient
from compas_fab.backends import MoveItPlanner
from compas_fab.robots import RobotCell
from compas_fab.robots import RigidBody

with RosClient() as client:
    robot = client.load_robot()
    planner = MoveItPlanner(client)

    # =========
    # Example 1
    # =========

    # Create a robot cell with a floor geometry
    robot_cell = RobotCell(robot)
    floor_mesh = Mesh.from_stl(compas_fab.get("planning_scene/floor.stl"))
    robot_cell.rigid_body_models["floor"] = RigidBody.from_mesh(floor_mesh)

    result = planner.set_robot_cell(robot_cell)
    print(result)

    # If you are running ROS with UI, you should see a floor in the PyBullet world
    input("Press Enter to continue...")

    # =========
    # Example 2
    # =========

    # Create another robot cell with a cone geometry
    robot_cell = RobotCell(robot)
    cone = Mesh.from_stl(compas_fab.get("planning_scene/cone.stl")).scaled(5)
    robot_cell.rigid_body_models["cone"] = RigidBody.from_mesh(cone)

    # This will replace the previous robot cell in the planner
    # There will not be any `floor`` in the PyBullet world
    result = planner.set_robot_cell(robot_cell)
    print(result)

    # If you are running ROS with UI, you should see a cone in the PyBullet world
    # and the floor should be gone
    input("Press Enter to continue...")
