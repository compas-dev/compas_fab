from compas.geometry import Frame
from compas_fab.backends import RosClient
from compas_fab.backends import MoveItPlanner

from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCellState

with RosClient() as client:
    robot = client.load_robot()
    assert robot.name == "ur5_robot"
    planner = MoveItPlanner(client)

    frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
    target = FrameTarget(frame_WCF)

    start_configuration = robot.zero_configuration()
    start_state = RobotCellState.from_robot_configuration(robot, start_configuration)

    configuration = planner.inverse_kinematics(target, start_state)

    print("Found configuration", configuration)
