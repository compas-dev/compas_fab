from compas.geometry import Frame
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner

from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCellLibrary

with PyBulletClient() as client:
    planner = PyBulletPlanner(client)
    robot_cell, robot_cell_state = RobotCellLibrary.ur5_cone_tool()
    planner.set_robot_cell(robot_cell)

    frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
    target = FrameTarget(frame_WCF)

    # The robot_cell_state from the RobotCellLibrary contains a zero configuration for the robot
    joint_positions, joint_names = planner.inverse_kinematics(target, robot_cell_state)

    print("Inverse kinematics result: ", joint_positions, joint_names)

    input("Observe the IK result in PyBullet's GUI, Press Enter to continue...")
