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
    options = {"max_results": 200}

    # Showing all unique solutions
    configuration_generator = planner.iter_inverse_kinematics(target, start_state)
    for i, configuration in enumerate(configuration_generator):
        print("Unique configuration {}: {}".format(i, configuration))
