from typing import Any, Dict
from compas.geometry import Box, Frame
from compas_fab.backends import InverseKinematicsError, MoveItPlanner, RosClient
from compas_fab.robots import FrameTarget, RigidBody, TargetMode

with RosClient() as client:
    planner = MoveItPlanner(client)

    # Create a default RobotCellState from the RobotCell as the starting state
    robot_cell = client.load_robot_cell()
    assert robot_cell.robot_model.name == "ur5_robot"

    # Add a collision geometry to the robot cell
    box_mesh = Box(0.1, 0.1, 0.6).to_mesh(triangulated=True)
    robot_cell.rigid_body_models["box"] = RigidBody.from_mesh(box_mesh)
    start_state = robot_cell.default_cell_state()
    start_state.rigid_body_states["box"].frame.point = [0.3, 0.1, 0.3]
    planner.set_robot_cell(robot_cell, start_state)

    # Create a target frame with the ROBOT mode
    frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
    target = FrameTarget(frame_WCF, TargetMode.ROBOT)
    print(f"Target frame with Robot Mode: {target}")

    # Calculate inverse kinematics
    def calculate(options: Dict[str, Any]) -> None:
        """Helper function to solve IK for the target frame and print the outcome."""
        try:
            configuration = planner.inverse_kinematics(target, start_state, options=options)
            print(f"    Found configuration: {configuration}")
        except InverseKinematicsError as e:
            print(f"    Failed to find a configuration: {e}")

    print("Inverse kinematics with collision check enabled: (by default)")
    calculate({})

    print("Inverse kinematics with collision check disabled:")
    options = {"check_collision": False}
    calculate(options)
