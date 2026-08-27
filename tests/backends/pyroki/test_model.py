import math

import pytest

pytest.importorskip("pyroki")

from compas.geometry import Quaternion
from compas.geometry import Frame
from compas_robots import Configuration
from compas_robots import RobotModel
from compas_robots.model import Axis
from compas_robots.model import Joint
from compas_robots.model import Limit
from compas_robots.model import Link
from compas_robots.model import Mimic

from compas_fab.backends.pyroki.model import robot_model_to_pyroki
from compas_fab.robots import RobotCellLibrary


def assert_frames_close(actual, expected, position_tolerance=1e-6, orientation_tolerance=1e-6):
    assert actual.point.distance_to_point(expected.point) <= position_tolerance
    actual_quaternion = Quaternion.from_frame(actual)
    expected_quaternion = Quaternion.from_frame(expected)
    dot = abs(
        actual_quaternion.w * expected_quaternion.w
        + actual_quaternion.x * expected_quaternion.x
        + actual_quaternion.y * expected_quaternion.y
        + actual_quaternion.z * expected_quaternion.z
    )
    assert 1.0 - min(1.0, dot) <= orientation_tolerance


@pytest.mark.parametrize(
    ("library_method", "joint_values"),
    [
        (RobotCellLibrary.ur5, [0.3, -0.4, 0.5, -0.6, 0.7, -0.8]),
        (RobotCellLibrary.panda, [0.2, -0.5, 0.3, -1.2, 0.4, 1.0, -0.7, 0.02]),
    ],
)
def test_forward_kinematics_matches_compas_for_every_link(library_method, joint_values):
    cell, state = library_method(load_geometry=False)
    state.robot_configuration.joint_values = joint_values
    model = robot_model_to_pyroki(cell.robot_model, state.robot_configuration)

    assert model.joint_names == tuple(state.robot_configuration.joint_names)
    for link in cell.robot_model.links:
        actual = model.forward_kinematics(state.robot_configuration, link.name)
        expected = cell.robot_model.forward_kinematics(state.robot_configuration, link.name)
        assert_frames_close(actual, expected)


def test_forward_kinematics_accepts_configuration_with_different_order():
    cell, state = RobotCellLibrary.ur5(load_geometry=False)
    model = robot_model_to_pyroki(cell.robot_model)
    configuration = state.robot_configuration
    configuration.joint_names.reverse()
    configuration.joint_values.reverse()

    actual = model.forward_kinematics(configuration, cell.get_end_effector_link_name())
    expected = cell.robot_model.forward_kinematics(configuration, cell.get_end_effector_link_name())
    assert_frames_close(actual, expected)


def test_forward_kinematics_supports_arbitrary_axes_prismatic_and_mimic_joints():
    links = [Link("base"), Link("rotated"), Link("translated"), Link("mimicked")]
    revolute = Joint(
        "revolute",
        Joint.REVOLUTE,
        "base",
        "rotated",
        origin=Frame([0.1, -0.2, 0.3], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]),
        axis=Axis("1 1 0"),
        limit=Limit(velocity=2.0, lower=-1.5, upper=1.5),
    )
    prismatic = Joint(
        "prismatic",
        Joint.PRISMATIC,
        "rotated",
        "translated",
        origin=Frame([0.2, 0.0, 0.1], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
        axis=Axis("0 1 1"),
        limit=Limit(velocity=0.5, lower=-0.2, upper=0.4),
    )
    mimicked = Joint(
        "mimicked_revolute",
        Joint.REVOLUTE,
        "translated",
        "mimicked",
        origin=Frame([0.0, 0.1, 0.2]),
        axis=Axis("0 0 1"),
        limit=Limit(velocity=2.0, lower=-1.5, upper=1.5),
        mimic=Mimic("revolute", multiplier=-0.5, offset=0.1),
    )
    robot = RobotModel("mixed_chain", joints=[prismatic, mimicked, revolute], links=links)
    configuration = Configuration([0.6, 0.15], [Joint.REVOLUTE, Joint.PRISMATIC], ["revolute", "prismatic"])
    model = robot_model_to_pyroki(robot, configuration)

    for link in links:
        actual = model.forward_kinematics(configuration, link.name)
        expected = robot.forward_kinematics(configuration, link.name)
        assert_frames_close(actual, expected)


def test_forward_kinematics_supports_continuous_and_fixed_joints():
    links = [Link("base"), Link("rotated"), Link("tip")]
    continuous = Joint(
        "continuous",
        Joint.CONTINUOUS,
        "base",
        "rotated",
        origin=Frame([0.1, 0.2, 0.3]),
        axis=Axis("0 1 0"),
    )
    fixed = Joint(
        "fixed",
        Joint.FIXED,
        "rotated",
        "tip",
        origin=Frame([0.0, 0.0, 0.4]),
    )
    robot = RobotModel("continuous_chain", joints=[fixed, continuous], links=links)
    configuration = Configuration([2.4], [Joint.CONTINUOUS], ["continuous"])
    model = robot_model_to_pyroki(robot, configuration)

    actual = model.forward_kinematics(configuration, "tip")
    expected = robot.forward_kinematics(configuration, "tip")
    assert_frames_close(actual, expected)
    assert float(model.robot.joints.lower_limits[0]) == pytest.approx(-math.pi)
    assert float(model.robot.joints.upper_limits[0]) == pytest.approx(math.pi)
