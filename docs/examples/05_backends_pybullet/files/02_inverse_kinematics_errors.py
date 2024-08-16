from compas.geometry import Frame
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner

from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCellLibrary

from compas_fab.backends.exceptions import InverseKinematicsError

with PyBulletClient() as client:

    # Load a pre-made robot cell with one tool from the RobotCellLibrary
    robot_cell, robot_cell_state = RobotCellLibrary.ur10e_gripper_one_beam()
    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)

    # The FrameTarget represents the tool's coordinate frame (TCF) when a tool is attached
    target_center_point = [0.0, 0.5, 0.15]
    frame_WCF = Frame(target_center_point, [1, 0, 0], [0, 1, 0])
    target = FrameTarget(frame_WCF)

    # Example 1: Default IK without collision checking
    # The target is causing the attached beam to collide with the floor.
    # However, the planner does not check for collisions and returns a solution.
    config = planner.inverse_kinematics(target, robot_cell_state)
    input("Observe the IK result in PyBullet's GUI, Press Enter to continue...")

    # Example 2: Enable collision checking in the IK
    try:
        # Enable the check_collision mode via options
        options = {"check_collision": True}
        config = planner.inverse_kinematics(target, robot_cell_state, options=options)
    except InverseKinematicsError as e:
        # The planner will try many times but still unable to find a solution
        # after "max_results", it will return InverseKinematicsError.
        print(e)
        print(e.message)
        print(e.target_pcf)
    input("Observe the IK result in PyBullet's GUI, Press Enter to continue...")
