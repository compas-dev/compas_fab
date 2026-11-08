from compas.geometry import Frame

from compas_fab.backends import MoveItPlanner
from compas_fab.backends import RosClient
from compas_fab.robots import FrameTarget
from compas_fab.robots import TargetMode

with RosClient() as client:
    planner = MoveItPlanner(client)

    # Create a default RobotCellState from the RobotCell as the starting state
    robot_cell = client.load_robot_cell()
    assert robot_cell.robot_model.name == "ur5_robot"
    start_state = robot_cell.default_cell_state()

    # Create a target frame with the ROBOT mode
    target = FrameTarget(Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0]), TargetMode.ROBOT)
    print("Target frame with Robot Mode: {}".format(target))

    # Calculate inverse kinematics
    configuration = planner.inverse_kinematics(target, start_state)
    print("Found configuration 1: ", configuration)

    # Calculate inverse kinematics with full configuration
    options = {"return_full_configuration": True}
    configuration = planner.inverse_kinematics(target, start_state, options=options)
    print("Found configuration 2: ", configuration)
