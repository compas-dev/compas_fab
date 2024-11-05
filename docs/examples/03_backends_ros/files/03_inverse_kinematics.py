from compas.geometry import Frame
from compas_fab.backends import RosClient
from compas_fab.backends import MoveItPlanner

from compas_fab.robots import FrameTarget
from compas_fab.robots import TargetMode

with RosClient() as client:
    robot_cell = client.load_robot_cell()
    assert robot_cell.robot_model.name == "ur5_robot"
    planner = MoveItPlanner(client)

    # Create a default RobotCellState from the RobotCell as the starting state
    start_state = robot_cell.default_cell_state()

    # Create a target frame with the ROBOT mode
    frame_WCF = Frame([0.3, 0.1, 0.5], [0, -1, 0], [0, 0, -1])
    target = FrameTarget(frame_WCF, TargetMode.ROBOT)
    print("Target frame with Robot Mode: {}".format(target))

    # Calculate inverse kinematics
    configuration = planner.inverse_kinematics(target, start_state)
    print("Found configuration", configuration)

    # Check that the forward kinematics of the found configuration matches the target frame
    start_state.robot_configuration = configuration
    frame = planner.forward_kinematics(start_state, TargetMode.ROBOT)
    print("Robot's Planner Coordinate Frame (PCF)")
    print(frame)
