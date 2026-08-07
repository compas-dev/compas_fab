from itertools import combinations

import pytest
from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Vector
from compas.tolerance import Tolerance

import compas_fab
from compas_fab.backends import AnalyticalKinematicsPlanner
from compas_fab.backends import UR5Kinematics
from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCellState
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode


# The tolerance for the tests are set to 1e-4 meters, equivalent to 0.1 mm
# Relative tolerance is set to 1e-3 (0.1%)
# Angular tolerance is set to 2e-3 radians, equivalent to 0.11 degrees
TOL = Tolerance(unit="M", absolute=1e-4, relative=1e-3, angular=2e-3)

urdf_filename = compas_fab.get("robot_library/ur5_robot/urdf/robot_description.urdf")
srdf_filename = compas_fab.get("robot_library/ur5_robot/robot_description_semantic.srdf")


@pytest.fixture
def ur5_planner_robot_only():
    solver = UR5Kinematics()
    planner = AnalyticalKinematicsPlanner(solver)

    # Set up the robot cell with only the robot
    # (No need to load the geometry because no CC)
    robot_cell, robot_cell_state = RobotCellLibrary.ur5(load_geometry=False)
    planner.set_robot_cell(robot_cell)

    # Set Initial RobotCellState
    planner.set_robot_cell_state(robot_cell_state)
    return planner


def test_forward_kinematics(ur5_planner_robot_only):
    planner = ur5_planner_robot_only  # type: AnalyticalKinematicsPlanner

    # Initial state of the robot cell is the home position
    start_state = planner.robot_cell.default_cell_state()
    frame = planner.forward_kinematics(start_state, TargetMode.ROBOT, group=None)

    # The frame of the `tool0` link at the zero configuration, i.e. the same answer the
    # URDF model gives. The solver's own chain ends at the flange, one fixed rotation
    # earlier, which is what `flange_frame` accounts for.
    correct = Frame(
        Point(x=0.81725, y=0.19145, z=-0.00549),
        Vector(x=-1.0, y=0.0, z=0.0),
        Vector(x=0.0, y=0.0, z=1.0),
    )
    assert TOL.is_allclose(frame.point, correct.point)
    assert TOL.is_allclose(frame.quaternion, correct.quaternion)

    # ...and it agrees with the robot model, which is what the scene objects draw
    robot_cell = planner.robot_cell
    model_frame = robot_cell.robot_model.forward_kinematics(start_state.robot_configuration, robot_cell.get_end_effector_link_name())
    assert TOL.is_allclose(frame.point, model_frame.point)
    assert TOL.is_allclose(frame.quaternion, model_frame.quaternion)


def test_iter_inverse_kinematics(ur5_planner_robot_only):
    planner = ur5_planner_robot_only  # type: AnalyticalKinematicsPlanner
    # This target has eight solutions (without CC)
    target = FrameTarget(Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269)), TargetMode.ROBOT)

    # The `iter_inverse_kinematics` method will return an iterator that yields all possible solutions
    start_state = planner.robot_cell.default_cell_state()
    solutions = list(planner.iter_inverse_kinematics(target, start_state, group=None))
    assert len(solutions) == 8
    # Every solution must actually reach the target, read through the robot model
    robot_cell = planner.robot_cell
    for configuration in solutions:
        state = start_state.copy()
        state.robot_configuration = configuration
        reached = robot_cell.forward_kinematics_target_frame(state, TargetMode.ROBOT)
        assert TOL.is_allclose(reached.point, target.target_frame.point)
        assert TOL.is_allclose(reached.quaternion, target.target_frame.quaternion)


def test_inverse_kinematics(ur5_planner_robot_only):
    planner = ur5_planner_robot_only  # type: AnalyticalKinematicsPlanner
    target = FrameTarget(Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269)), TargetMode.ROBOT)
    start_state = planner.robot_cell.default_cell_state()

    # Test to confirm inverse_kinematics() return one solution at a time
    solutions = []  # type: list[Configuration]
    for i in range(8):
        solutions.append(planner.inverse_kinematics(target, start_state, group=None))

    # Check First Solution Correctness by reading it back through the robot model
    robot_cell = planner.robot_cell
    state = start_state.copy()
    state.robot_configuration = solutions[0]
    reached = robot_cell.forward_kinematics_target_frame(state, TargetMode.ROBOT)
    assert TOL.is_allclose(reached.point, target.target_frame.point)
    assert TOL.is_allclose(reached.quaternion, target.target_frame.quaternion)

    # Check that all solutions are different
    for a, b in combinations(solutions, 2):
        assert not TOL.is_allclose(a.joint_values, b.joint_values)


def forward_inverse_agreement(planner, start_state):
    """Helper function to test forward and inverse kinematics agreement"""

    # First perform forward kinematics to find the frame
    starting_configuration = start_state.robot_configuration
    frame = planner.forward_kinematics(start_state, TargetMode.ROBOT, group=None)

    # Second, perform Inverse Kinematics to see if any of the solutions
    # are the same as the original joint configuration
    # Because there are 8 possible solutions, we can break early if we find a close solution

    start_state = RobotCellState.from_robot_cell(planner.client.robot_cell)
    for configuration in planner.iter_inverse_kinematics(FrameTarget(frame, TargetMode.ROBOT), start_state, group=None):
        if configuration.close_to(starting_configuration):
            print("Found a close solution: {}".format(configuration))
            return True
    assert False


def test_forward_inverse_agreement_ur5(ur5_planner_robot_only):
    """Test that the forward and inverse kinematics are in agreement"""
    # It is a known problem that the kinematics in AnalyticalKinematicsPlanner does not always
    # return the same result as the kinematics in the RobotModel.

    # TODO: This problem will be solved after we improved the AnalyticalKinematics class
    # with base and tip offsets, as well as joint value offsets.
    # This will allow us to compare the results of the AnalyticalKinematics with the RobotModel

    planner = ur5_planner_robot_only  # type: AnalyticalKinematicsPlanner

    # This test starts with FK and then uses IK to find the original joint configuration
    start_state = RobotCellState.from_robot_cell(planner.client.robot_cell)

    # All these joint values should be positive and smaller than 2*pi
    start_state.robot_configuration.joint_values = [0.2, 0.5, 1.4, 1.3, 2.6, 2.3]
    forward_inverse_agreement(planner, start_state)

    start_state.robot_configuration.joint_values = [0.6, 0.5, 1.4, 1.3, 2.6, 2.3]
    forward_inverse_agreement(planner, start_state)

    start_state.robot_configuration.joint_values = [0.6, 2.5, 1.4, 1.3, 2.6, 2.3]
    forward_inverse_agreement(planner, start_state)

    start_state.robot_configuration.joint_values = [0.6, 2.5, 3.4, 1.3, 2.6, 2.3]
    forward_inverse_agreement(planner, start_state)

    start_state.robot_configuration.joint_values = [0.6, 2.5, 3.4, 4.3, 2.6, 2.3]
    forward_inverse_agreement(planner, start_state)

    start_state.robot_configuration.joint_values = [0.6, 2.5, 3.4, 4.3, 4.6, 2.3]
    forward_inverse_agreement(planner, start_state)

    start_state.robot_configuration.joint_values = [0.6, 2.5, 3.4, 4.3, 4.6, 4.3]
    forward_inverse_agreement(planner, start_state)
