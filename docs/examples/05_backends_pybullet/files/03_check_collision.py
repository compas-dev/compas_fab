import time

import compas_fab
from compas_robots import Configuration
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import RobotCellLibrary
from compas_fab.backends import CollisionCheckError

with PyBulletClient("gui") as client:
    planner = PyBulletPlanner(client)
    print("Observe the Pybullet GUI window to see the robot cell state being checked")

    # The robot cell is loaded from RobotCellLibrary
    robot_cell, robot_cell_state = RobotCellLibrary.ur10e_gripper_one_beam()
    planner.set_robot_cell(robot_cell)

    # ---------------------------------------------
    # Example 1 - No collision
    # ---------------------------------------------

    # This configuration is not in collision
    robot_cell_state.robot_configuration.joint_values = [0, -1.5, 2.0, 0, 0, 0]

    # The following check_collision should pass without raising an exception
    start = time.time()
    planner.check_collision(robot_cell_state)
    print("Time taken for collision check: {}".format(time.time() - start))
    input("Press Enter to continue...\n")

    # ---------------------------------------------
    # Example 2 - Collision between beam and floor
    # ---------------------------------------------

    robot_cell_state.robot_configuration.joint_values = [0, -1.0, 2.0, 0, 0, 0]

    # The following check_collision should raise an exception
    try:
        planner.check_collision(robot_cell_state)
    except CollisionCheckError as e:
        print("\nCollision detected: \n{}".format(e))
    input("Press Enter to continue...\n")

    # ---------------------------------------------
    # Example 3 - Multiple Collisions
    # ---------------------------------------------

    robot_cell_state.robot_configuration.joint_values = [0, -0.8, 2.0, 0, 0, 0]

    # The following check_collision should raise an exception
    # The `full_report` option will return all failed collision pairs in the exception message
    try:
        planner.check_collision(robot_cell_state, options={"full_report": True})
    except CollisionCheckError as e:
        print("\nCollision detected: \n{}".format(e))
    input("Press Enter to continue...\n")

    # ---------------------------------------------
    # Example 4 - Verbose Mode
    # ---------------------------------------------

    robot_cell_state.robot_configuration.joint_values = [0, -0.7, 2.0, 0, 0, 0]

    # The following check_collision should raise an exception
    # The `full_report` option will print all failed collision pairs`
    try:
        planner.check_collision(robot_cell_state, options={"verbose": True, "full_report": True})
    except CollisionCheckError as e:
        print("\nCollision detected: \n{}".format(e))
    input("Press Enter to continue...\n")

    # The verbose action will print all tested collision pairs
