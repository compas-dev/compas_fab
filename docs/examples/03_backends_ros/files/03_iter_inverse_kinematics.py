from compas.geometry import Frame
from compas_fab.backends import RosClient
from compas_fab.backends import MoveItPlanner

from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCellState
from compas_fab.robots import TargetMode

with RosClient() as client:
    robot_cell = client.load_robot_cell()
    assert robot.name == "ur5_robot"
    planner = MoveItPlanner(client)

    frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
    target = FrameTarget(frame_WCF, TargetMode.ROBOT)

    start_configuration = robot.zero_configuration()
    start_state = RobotCellState.from_robot_configuration(robot, start_configuration)

    result_count = 0
    for config in planner.iter_inverse_kinematics(target, start_state, options=dict(max_results=10)):
        print("Found configuration", config)
        result_count += 1
    print("Found %d configurations" % result_count)
