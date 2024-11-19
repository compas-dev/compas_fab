from compas.geometry import Frame
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner

from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import RigidBodyLibrary
from compas_fab.robots import RigidBodyState
from compas_fab.robots import TargetMode

from compas_fab.backends.exceptions import InverseKinematicsError

with PyBulletClient() as client:

    # Load a pre-made robot cell with one tool from the RobotCellLibrary
    robot_cell, robot_cell_state = RobotCellLibrary.ur10e_gripper_one_beam()
    planner = PyBulletPlanner(client)

    # Load a target marker for illustration
    target_marker = RigidBodyLibrary.target_marker(0.5)
    robot_cell.rigid_body_models["target_marker"] = target_marker
    planner.set_robot_cell(robot_cell)

    # Choose TargetMode.WORKPIECE for the FrameTarget to directly specify the beam's location
    beam_target_point = [0.0, 0.5, 0.01]
    frame_WCF = Frame(beam_target_point, [0, 0, -1], [-1, 0, 0])
    robot_cell_state.rigid_body_states["target_marker"] = RigidBodyState(frame_WCF)
    target = FrameTarget(frame_WCF, TargetMode.WORKPIECE)

    # ----------------------------------------------
    # Example 1: IK without collision checking
    # ----------------------------------------------

    # The target is causing the attached beam to collide with the floor.
    # However, if the planner does not check for collisions, it will returns a solution.
    options = {"check_collision": False}
    config = planner.inverse_kinematics(target, robot_cell_state, options=options)
    input("Observe the IK result in PyBullet's GUI, Press Enter to continue...")

    # ----------------------------------------------
    # Example 2: Enable collision checking in the IK
    # ----------------------------------------------
    try:
        # Enable the check_collision mode via options
        options = {"check_collision": True, "max_results": 1000}  # Default is True
        config = planner.inverse_kinematics(target, robot_cell_state, options=options)
    except InverseKinematicsError as e:
        # The planner will try many times but still unable to find a solution
        # after "max_results", it will return InverseKinematicsError.
        print(e)
        print(e.message)
        print(e.target_pcf)
    input("Observe the IK result in PyBullet's GUI, Press Enter to continue...")
