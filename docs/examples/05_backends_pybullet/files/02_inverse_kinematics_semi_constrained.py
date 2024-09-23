# This example demonstrates the semi-constrained mode of the PyBullet Inverse Kinematics (IK) solver.
# The semi-constrained mode will cause the IK solver to only the target's point position and ignore the target's orientation.
# It is only possible to use a Target with TargetMode.ROBOT in this mode.

from compas.geometry import Frame
from compas.geometry import distance_point_point

from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner

from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import RigidBodyLibrary
from compas_fab.robots import RobotCellState
from compas_fab.robots import RobotLibrary
from compas_fab.robots import TargetMode

# NOTE: The semi-constrained IK mode cannot be used with tools

with PyBulletClient() as client:
    # Create a robot cell with a UR5 robot
    robot = RobotLibrary.ur5()
    robot_cell = RobotCell(robot)

    # Add a target marker to visualize the target
    robot_cell.rigid_body_models["target_marker"] = RigidBodyLibrary.target_marker(size=0.15)

    robot_cell_state = RobotCellState.from_robot_cell(robot_cell)

    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)

    # The initial configuration changes the IK result
    start_configuration = robot_cell.robot.zero_configuration()

    # Note that the semi-constrained IK mode only accepts ROBOT mode.
    frame_WCF = Frame([0.3, 0.1, 0.5], [1, 1, 0], [0, 0, 1])
    target = FrameTarget(frame_WCF, TargetMode.ROBOT)

    # Place the target marker at the target's frame
    robot_cell_state.rigid_body_states["target_marker"].frame = frame_WCF

    print(" ")

    # =============================
    # IK with semi-constrained mode
    # =============================

    config = planner.inverse_kinematics(target, robot_cell_state, options={"semi-constrained": True})

    print("Inverse kinematics result: ", config)

    result_state = robot_cell_state.copy()  # type: RobotCellState
    result_state.robot_configuration = config

    # Perform forward kinematics to verify the result
    fk_frame = planner.forward_kinematics(result_state, TargetMode.ROBOT)
    print("Forward kinematics frame: \n", fk_frame)
    distance_to_target = distance_point_point(fk_frame.point, target.target_frame.point)
    assert distance_to_target < PyBulletPlanner.DEFAULT_TARGET_TOLERANCE_POSITION
    print(
        "Distance to target: {} is smaller than DEFAULT_TARGET_TOLERANCE_POSITION({})".format(
            distance_to_target, PyBulletPlanner.DEFAULT_TARGET_TOLERANCE_POSITION
        )
    )

    # The FK result above shows that only the position of the frame matches with the target at Point(x=0.300, y=0.100, z=0.500)
    # The orientation of the frame is arbitrary

    input("Observe the IK result in PyBullet's GUI, Press Enter to continue...")
    print(" ")

    # =============================
    # IK in normal operation
    # =============================

    config = planner.inverse_kinematics(target, robot_cell_state)

    print("Inverse kinematics result: ", config)

    result_state = robot_cell_state.copy()  # type: RobotCellState
    result_state.robot_configuration = config

    # Perform forward kinematics to verify the result
    fk_frame = planner.forward_kinematics(result_state, TargetMode.ROBOT)
    print("Forward kinematics frame: \n", fk_frame)

    # The FK result above shows that the position and orientation of the frame matches with the target
    input("Observe the IK result in PyBullet's GUI, Press Enter to continue...")
