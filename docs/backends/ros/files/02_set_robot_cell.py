from compas_fab.backends import RosClient
from compas_fab.backends import MoveItPlanner

from compas_fab.robots import ToolLibrary
from compas_fab.robots import RigidBody
from compas_fab.robots import RigidBodyLibrary

from compas.geometry import Box

with RosClient() as client:
    robot_cell = client.load_robot_cell()
    planner = MoveItPlanner(client)

    # Add a gripper to the robot cell
    gripper = ToolLibrary.static_gripper_small()
    robot_cell.tool_models["my_gripper"] = gripper

    # Add a beam to the robot cell
    # Use triangulated mesh
    beam_mesh = Box(1, 0.1, 0.1).to_mesh(triangulated=True)
    beam = RigidBody.from_mesh(beam_mesh)
    robot_cell.rigid_body_models["my_beam"] = beam

    # Add a floor to the robot cell
    floor = RigidBodyLibrary.floor()
    robot_cell.rigid_body_models["my_floor"] = floor

    # Set the robot cell back to the client
    planner.set_robot_cell(robot_cell)
    # If you are running ROS with UI, you should see the objects in the PyBullet world
    input("Press Enter to terminate...")
