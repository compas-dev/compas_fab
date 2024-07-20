import compas_fab
from compas.geometry import Frame
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner

from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import RobotLibrary

with PyBulletClient() as client:
    robot = RobotLibrary.ur5()
    planner = PyBulletPlanner(client)
    planner.set_robot_cell(RobotCell(robot))

    frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
    target = FrameTarget(frame_WCF)

    start_configuration = robot.zero_configuration()
    start_state = RobotCellState.from_robot_configuration(robot, start_configuration)

    configuration = planner.inverse_kinematics(target, start_state)

    print("Found configuration", configuration)
