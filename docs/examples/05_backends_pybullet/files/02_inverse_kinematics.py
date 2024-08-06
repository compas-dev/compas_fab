from compas.geometry import Frame
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner

from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import RobotLibrary

with PyBulletClient() as client:
    planner = PyBulletPlanner(client)
    robot = RobotLibrary.ur5()
    planner.set_robot_cell(RobotCell(robot))

    frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
    target = FrameTarget(frame_WCF)

    start_configuration = robot.zero_configuration()
    robot_cell_state = RobotCellState.from_robot_configuration(robot, start_configuration)

    joint_positions, joint_names = planner.inverse_kinematics(target, robot_cell_state)

    print("Inverse kinematics result: ", joint_positions, joint_names)

    input("Observe the IK result in PyBullet's GUI, Press Enter to continue...")
