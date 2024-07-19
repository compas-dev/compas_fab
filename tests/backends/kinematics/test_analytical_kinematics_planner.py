import pytest

from itertools import combinations
import compas
from compas.tolerance import Tolerance
from compas.geometry import Frame
from compas.geometry import Vector
from compas.geometry import Point
from compas_robots import Configuration

import compas_fab
from compas_fab.backends import AnalyticalKinematicsPlanner
from compas_fab.backends import UR5Kinematics
from compas_fab.robots import Tool
from compas_fab.robots import RobotLibrary
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import RobotCellState
from compas_fab.robots import RobotCell
from compas_fab.robots import FrameTarget


if not compas.IPY:
    from compas_fab.backends import AnalyticalPyBulletClient


# The tolerance for the tests are set to 1e-4 meters, equivalent to 0.1 mm
# Relative tolerance is set to 1e-3 (0.1%)
# Angular tolerance is set to 2e-3 radians, equivalent to 0.11 degrees
TOL = Tolerance(unit="m", absolute=1e-4, relative=1e-3, angular=2e-3)

urdf_filename = compas_fab.get("robot_library/ur5_robot/urdf/robot_description.urdf")
srdf_filename = compas_fab.get("robot_library/ur5_robot/robot_description_semantic.srdf")


@pytest.fixture
def ur5_planner_no_geo():
    solver = UR5Kinematics()
    planner = AnalyticalKinematicsPlanner(solver)

    # Set up the robot cell
    robot = RobotLibrary.ur5(load_geometry=False)  # No need to load the geometry because no CC
    robot_cell = RobotCell(robot)
    planner.set_robot_cell(robot_cell)
    return planner


def test_forward_kinematics(ur5_planner_no_geo):
    start_state = RobotCellState.from_robot_cell(ur5_planner_no_geo.robot_cell)
    frame = ur5_planner_no_geo.forward_kinematics(start_state, group=None)
    correct = Frame(
        Point(x=0.81725, y=0.19145, z=-0.00549),
        Vector(x=0.0, y=1.0, z=-0.0),
        Vector(x=1.0, y=-0.0, z=-0.0),
    )
    assert TOL.is_allclose(frame.point, correct.point)
    assert TOL.is_allclose(frame.quaternion, correct.quaternion)


def test_iter_inverse_kinematics(ur5_planner_no_geo):

    # This target has eight solutions (without CC)
    target = FrameTarget(Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269)))

    # The `iter_inverse_kinematics` method will return an iterator that yields all possible solutions
    start_state = RobotCellState.from_robot_cell(ur5_planner_no_geo.robot_cell)
    solutions = list(ur5_planner_no_geo.iter_inverse_kinematics(target, start_state, group=None))
    assert len(solutions) == 8
    joint_positions, _ = solutions[0]
    correct = Configuration.from_revolute_values((0.022, 4.827, 1.508, 1.126, 1.876, 3.163))
    assert correct.close_to(Configuration.from_revolute_values(joint_positions))


def test_inverse_kinematics(ur5_planner_no_geo):
    target = FrameTarget(Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269)))
    start_state = RobotCellState.from_robot_cell(ur5_planner_no_geo.robot_cell)

    # Test to confirm inverse_kinematics() return one solution at a time
    solutions = []
    for i in range(8):
        solutions.append(ur5_planner_no_geo.inverse_kinematics(target, start_state, group=None))

    # Check First Solution Correctness
    first_solution = solutions[0]
    joint_positions, _ = first_solution
    correct = Configuration.from_revolute_values((0.022, 4.827, 1.508, 1.126, 1.876, 3.163))
    assert correct.close_to(Configuration.from_revolute_values(joint_positions))

    # Check that all solutions are different
    for a, b in combinations(solutions, 2):
        assert not TOL.is_allclose(a[0], b[0])


def forward_inverse_agreement(planner, start_state):
    # Helper function to test forward and inverse kinematics agreement
    starting_configuration = start_state.robot_configuration
    frame = planner.forward_kinematics(start_state, group=None)

    # Checks if one of the IK results is the same as the original joint configuration
    close_soliution_found = False
    start_state = RobotCellState.from_robot_cell(planner.robot_cell)
    for solutions in planner.iter_inverse_kinematics(FrameTarget(frame), start_state, group=None):
        joint_positions, _ = solutions
        if Configuration.from_revolute_values(joint_positions).close_to(starting_configuration):
            close_soliution_found = True
            print("Found a close solution: {}".format(joint_positions))
            break
    assert close_soliution_found


def test_forward_inverse_agreement_ur5(ur5_planner_no_geo):
    # Test that the forward and inverse kinematics are in agreement

    # This test starts with FK and then uses IK to find the original joint configuration
    start_state = RobotCellState.from_robot_cell(ur5_planner_no_geo.robot_cell)

    # All these joint values should be positive and smaller than 2*pi
    start_state.robot_configuration.joint_values = [0.2, 0.5, 1.4, 1.3, 2.6, 2.3]
    forward_inverse_agreement(ur5_planner_no_geo, start_state)

    start_state.robot_configuration.joint_values = [0.6, 0.5, 1.4, 1.3, 2.6, 2.3]
    forward_inverse_agreement(ur5_planner_no_geo, start_state)

    start_state.robot_configuration.joint_values = [0.6, 2.5, 1.4, 1.3, 2.6, 2.3]
    forward_inverse_agreement(ur5_planner_no_geo, start_state)

    start_state.robot_configuration.joint_values = [0.6, 2.5, 3.4, 1.3, 2.6, 2.3]
    forward_inverse_agreement(ur5_planner_no_geo, start_state)

    start_state.robot_configuration.joint_values = [0.6, 2.5, 3.4, 4.3, 2.6, 2.3]
    forward_inverse_agreement(ur5_planner_no_geo, start_state)

    start_state.robot_configuration.joint_values = [0.6, 2.5, 3.4, 4.3, 4.6, 2.3]
    forward_inverse_agreement(ur5_planner_no_geo, start_state)

    start_state.robot_configuration.joint_values = [0.6, 2.5, 3.4, 4.3, 4.6, 4.3]
    forward_inverse_agreement(ur5_planner_no_geo, start_state)
