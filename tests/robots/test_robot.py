import os
import re

import pytest
from compas.geometry import Frame
from compas.robots import RobotModel

from compas.robots.base_artist import BaseRobotModelArtist

from compas_fab.robots import Robot
from compas_fab.robots import RobotSemantics
from compas_fab.robots.ur5 import Robot as Ur5Robot

BASE_FOLDER = os.path.dirname(__file__)


@pytest.fixture
def panda_srdf():
    return os.path.join(BASE_FOLDER, 'fixtures', 'panda_semantics.srdf')


@pytest.fixture
def panda_urdf():
    return os.path.join(BASE_FOLDER, 'fixtures', 'panda.urdf')


@pytest.fixture
def panda_robot_instance(panda_urdf, panda_srdf):
    model = RobotModel.from_urdf_file(panda_urdf)
    semantics = RobotSemantics.from_srdf_file(panda_srdf, model)
    return Robot(model, semantics=semantics)


@pytest.fixture
def panda_robot_instance_w_artist(panda_robot_instance):
    robot = panda_robot_instance
    robot.artist = BaseRobotModelArtist(robot.model)

    return robot


@pytest.fixture
def panda_robot_instance_wo_semantics(panda_urdf):
    return Robot(RobotModel.from_urdf_file(panda_urdf), semantics=None)


@pytest.fixture
def ur5_robot_instance():
    return Ur5Robot()


@pytest.fixture
def ur5_joints(ur5_robot_instance):
    return ur5_robot_instance.model.joints


@pytest.fixture
def ur5_links(ur5_robot_instance):
    return ur5_robot_instance.model.links


def test_basic_name_only():
    robot = Robot.basic('testbot')
    assert robot.artist is None


def test_basic_name_joints_links(ur5_joints, ur5_links):
    robot = Robot.basic('testbot', joints=ur5_joints, links=ur5_links)
    assert len(robot.model.links) == 11
    assert len(robot.get_configurable_joint_names()) == 6


def test_basic_name_joints(ur5_joints):
    robot = Robot.basic('testbot', joints=ur5_joints)
    assert len(robot.model.joints) == 10


def test_basic_name_links(ur5_links):
    robot = Robot.basic('testbot', links=ur5_links)
    assert len(robot.model.links) == 11


def test_basic_attr():
    robot = Robot.basic('testbot', location='rfl')
    assert robot.model.attr['location'] == 'rfl'


def test_name(panda_robot_instance):
    robot = panda_robot_instance
    assert robot.name == 'panda'


def test_group_names(ur5_robot_instance):
    robot = ur5_robot_instance
    assert robot.group_names == ['manipulator', 'endeffector']


def test_main_group_name(panda_robot_instance):
    robot = panda_robot_instance
    assert robot.main_group_name == 'panda_arm_hand'


def test_root_name(ur5_robot_instance):
    robot = ur5_robot_instance
    assert robot.root_name == 'world'


def test_get_end_effector_link_name(panda_robot_instance):
    robot = panda_robot_instance
    assert robot.get_end_effector_link_name(group=None) == 'panda_rightfinger'
    assert robot.get_end_effector_link_name(group='panda_arm_hand') == 'panda_rightfinger'
    assert robot.get_end_effector_link_name(group='panda_arm') == 'panda_link8'


def test_get_end_effector_link_name_wrong_group(panda_robot_instance):
    robot = panda_robot_instance
    with pytest.raises(KeyError):
        robot.get_end_effector_link_name(group='panda_leg')


def test_get_end_effector_link(ur5_robot_instance):
    robot = ur5_robot_instance

    assert robot.get_end_effector_link(group=None).name == "ee_link"
    assert robot.get_end_effector_link(group='endeffector').name == "ee_link"


def test_get_end_effector_frame(panda_robot_instance):
    robot = panda_robot_instance
    assert round(robot.get_end_effector_frame(group=None).point.x, 3) == .301
    assert round(robot.get_end_effector_frame(group='panda_arm').point.x, 3) == .359


def test_get_base_link_name(ur5_robot_instance):
    robot = ur5_robot_instance

    assert robot.get_base_link_name(group=None) == 'base_link'
    assert robot.get_base_link_name(group='endeffector') == 'ee_link'


def test_get_base_link_name_wo_semantics(panda_robot_instance_wo_semantics):
    robot = panda_robot_instance_wo_semantics

    assert robot.get_base_link_name(group=None) == 'panda_link0'
    assert robot.get_base_link_name(group='endeffector') == 'panda_link0'


def test_get_base_link(panda_robot_instance):
    robot = panda_robot_instance

    link = robot.get_base_link(group=None)
    assert link.name == 'panda_link0'

    link = robot.get_base_link(group='panda_arm')
    assert link.name == 'panda_link0'

    link = robot.get_base_link(group='hand')
    assert link.name == 'panda_hand'


def test_get_base_link_wo_semantics(panda_robot_instance_wo_semantics):
    robot = panda_robot_instance_wo_semantics

    link = robot.get_base_link(group=None)
    assert link.name == 'panda_link0'

    link = robot.get_base_link(group='panda_arm')
    assert link.name == 'panda_link0'

    link = robot.get_base_link(group='hand')
    assert link.name == 'panda_link0'


def test_get_base_frame(panda_robot_instance):
    robot = panda_robot_instance

    assert robot.get_base_frame(group=None) == Frame.worldXY()
    assert robot.get_base_frame(group='panda_arm') == Frame.worldXY()


def test_get_base_frame_w_artist(panda_robot_instance_w_artist):
    robot = panda_robot_instance_w_artist

    assert robot.get_base_frame(group=None) == Frame.worldXY()
    assert robot.get_base_frame(group='panda_arm') == Frame.worldXY()


def test_get_base_frame_when_link_has_parent(ur5_robot_instance):
    robot = ur5_robot_instance
    base_frame = robot.get_base_frame(group='endeffector')

    assert [round(v, 3) for v in list(base_frame.point)] == [0.817, 0.191, -0.005]
    assert [round(v) for v in base_frame.data['xaxis']] == [0, 1, 0]
    assert [round(v) for v in base_frame.data['yaxis']] == [1, 0, 0]


def test_get_configurable_joints(ur5_robot_instance):
    robot = ur5_robot_instance
    joints = robot.get_configurable_joints()

    pattern = re.compile(r'(shoulder|wrist|elbow).*_joint')
    matches = [pattern.match(joint.name) for joint in joints]
    assert all(matches)


def test_get_configurable_joints_wo_semantics(panda_robot_instance_wo_semantics):
    robot = panda_robot_instance_wo_semantics
    joints = robot.get_configurable_joints()

    pattern = re.compile(r'panda_.*joint\d')
    matches = [pattern.match(joint.name) for joint in joints]
    assert all(matches)
