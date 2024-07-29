import time

import compas_fab
from compas_robots import Configuration
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import RobotCellLibrary


# #############################################
# Pybullet Check Collision Example
# #############################################


with PyBulletClient("gui") as client:
    planner = PyBulletPlanner(client)

    # The robot cell is loaded from RobotCellLibrary
    robot_cell, robot_cell_state = RobotCellLibrary.ur10e_gripper_one_beam()
    planner.set_robot_cell(robot_cell)

    # This configuration is not in collision
    robot_cell_state.robot_configuration.joint_values = [0, -2.0, 2.0, 0, 0, 0]

    # The following check_collision should pass without raising an exception
    start = time.time()
    planner.check_collision(robot_cell_state)
    print(f"Time taken for collision check: {time.time() - start}")
    input("Press Enter to continue...")

    # This configuration is in collision
    robot_cell_state.robot_configuration.joint_values = [0, 0.0, 2.0, 0, 0, 0]

    # The following check_collision should raise an exception
    # The verbose action will print all tested collision pairs
    try:
        planner.check_collision(robot_cell_state, options={"verbose": True})
    except Exception as e:
        print(f"Collision detected: {e}")
    input("Press Enter to continue...")
