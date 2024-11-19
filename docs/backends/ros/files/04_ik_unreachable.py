from compas.geometry import Frame

from compas_fab.backends import MoveItPlanner
from compas_fab.backends import RosClient
from compas_fab.robots import FrameTarget
from compas_fab.robots import TargetMode

from compas_fab.backends import InverseKinematicsError

with RosClient() as client:
    planner = MoveItPlanner(client)

    # Create a default RobotCellState from the RobotCell as the starting state
    robot_cell = client.load_robot_cell()
    assert robot_cell.robot_model.name == "ur5_robot"
    start_state = robot_cell.default_cell_state()

    # Create a target frame far away that is out of reach
    frame_WCF = Frame([2.0, 1.0, 1.0], [1, 0, 0], [0, 1, 0])
    target = FrameTarget(frame_WCF, TargetMode.ROBOT)

    # Calculate inverse kinematics
    try:
        configuration = planner.inverse_kinematics(target, start_state)
    except InverseKinematicsError as e:
        print("Failed to find a configuration: ", e)

"""
Output:
>>> Failed to find a configuration:  No inverse kinematics solution found.
"""
