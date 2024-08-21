from compas.geometry import Frame
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner

from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCellLibrary

with PyBulletClient() as client:

    # Load a pre-made robot cell with one tool from the RobotCellLibrary
    robot_cell, robot_cell_state = RobotCellLibrary.ur5_cone_tool()
    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)

    # The FrameTarget represents the tool's coordinate frame (TCF) when a tool is attached
    target_center_point = [0.0, 0.5, 0.5]
    frame_WCF = Frame(target_center_point, [1, 0, 0], [0, 0, -1])
    target = FrameTarget(frame_WCF)

    config = planner.inverse_kinematics(target, robot_cell_state)

    print("Inverse kinematics result: ", config)

    input("Observe the IK result in PyBullet's GUI, Press Enter to continue...")

    # The planner automatically plans such that the tool tip point stays on the target frame
    # In PyBullet's GUI, notice the tool's tip stays at the same location, only the orientation changes

    frame_WCFs = []
    frame_WCFs.append(Frame(target_center_point, [1, 1, 0], [0, 0, -1]))
    frame_WCFs.append(Frame(target_center_point, [0, 1, 0], [0, 0, -1]))
    frame_WCFs.append(Frame(target_center_point, [-1, 1, 0], [0, 0, -1]))
    frame_WCFs.append(Frame(target_center_point, [-1, 0, 0], [0, 0, -1]))
    frame_WCFs.append(Frame(target_center_point, [-1, -1, 0], [0, 0, -1]))
    frame_WCFs.append(Frame(target_center_point, [0, -1, 0], [0, 0, -1]))

    for frame_WCF in frame_WCFs:
        target = FrameTarget(frame_WCF)
        config = planner.inverse_kinematics(target, robot_cell_state)
        print("Inverse kinematics result: ", config)
        input("Observe the IK result in PyBullet's GUI, Press Enter to continue...")
