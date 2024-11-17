from compas.geometry import Frame

from compas_fab.backends import MoveItPlanner
from compas_fab.backends import RosClient
from compas_fab.robots import RobotCellLibrary

with RosClient() as client:
    planner = MoveItPlanner(client)

    # =========
    # Step 1
    # =========

    # Load a robot cell that contains a gripper and a beam
    robot_cell, _ = RobotCellLibrary.ur5_gripper_one_beam()
    gripper = robot_cell.tool_models["gripper"]
    beam = robot_cell.rigid_body_models["beam"]

    # The objects are simply added to the backend, their positions are not set
    planner.set_robot_cell(robot_cell)

    # If you are running ROS with UI, you should see the added object
    # positioned around the world origin
    input("Press Enter to continue...")

    # =========
    # Step 2
    # =========
    # Set the position of the beam
    robot_cell_state = robot_cell.default_cell_state()
    robot_cell_state.rigid_body_states["beam"].frame = Frame([1.2, 0, 0], [0, 0, 1], [1, 0, 0])

    # Attach the gripper to the robot
    robot_cell_state.tool_states["gripper"].attached_to_group = robot_cell.main_group_name
    robot_cell_state.tool_states["gripper"].attachment_frame = Frame([0, 0, 0], [0, 0, 1], [1, 0, 0])

    # Set the robot cell state in the planner
    planner.set_robot_cell_state(robot_cell_state)

    # If you are running ROS with UI, you should see the gripper attached to the robot
    input("Press Enter to continue...")

    # =========
    # Step 3
    # =========

    # Attach the beam to the robot
    robot_cell_state.rigid_body_states["beam"].attached_to_tool = "gripper"
    robot_cell_state.rigid_body_states["beam"].attachment_frame = Frame([0, 0, 0], [1, 0, 0], [0, 1, 0])

    # Set the robot cell state in the planner
    planner.set_robot_cell_state(robot_cell_state)

    # If you are running ROS with UI, you should see the beam attached to the gripper
    input("Press Enter to continue...")

    # =========
    # Step 4
    # =========

    # Move the robot to a specific configuration
    robot_cell_state.robot_configuration._joint_values[1] = -1.0
    robot_cell_state.robot_configuration._joint_values[2] = 0.5
    planner.set_robot_cell_state(robot_cell_state)

    # If you are running ROS with UI, you should see the robot in a different configuration
    # where the gripper and the beam are still attached to the robot.
    input("Press Enter to terminate...")
