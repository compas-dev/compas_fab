import os

import pytest
from compas.robots import RobotModel

from compas_fab.robots import RobotSemantics
from compas_fab.robots.ur5 import Robot as Ur5Robot

BASE_FOLDER = os.path.dirname(__file__)


@pytest.fixture
def panda_srdf():
    return os.path.join(BASE_FOLDER, 'fixtures', 'panda_semantics.srdf')


@pytest.fixture
def panda_urdf():
    return os.path.join(BASE_FOLDER, 'fixtures', 'panda.urdf')


def test_panda_srdf_file(panda_srdf, panda_urdf):
    model = RobotModel.from_urdf_file(panda_urdf)
    semantics = RobotSemantics.from_srdf_file(panda_srdf, model)
    assert semantics.group_names == ['panda_arm', 'hand', 'panda_arm_hand']
    assert semantics.main_group_name == 'panda_arm'
    assert semantics.get_base_link_name() == 'panda_link0'
    assert semantics.get_end_effector_link_name() == 'panda_link8'
    assert semantics.get_configurable_joint_names() == ['panda_joint1',
                                                        'panda_joint2',
                                                        'panda_joint3',
                                                        'panda_joint4',
                                                        'panda_joint5',
                                                        'panda_joint6',
                                                        'panda_joint7']


def test_ur5_semantics():
    robot = Ur5Robot()
    semantics = robot.semantics
    assert semantics.group_names == ['manipulator', 'endeffector']
    assert semantics.main_group_name == 'manipulator'
    assert semantics.get_base_link_name() == 'base_link'
    assert semantics.get_end_effector_link_name() == 'ee_link'
    assert semantics.get_configurable_joint_names() == ['shoulder_pan_joint',
                                                        'shoulder_lift_joint',
                                                        'elbow_joint',
                                                        'wrist_1_joint',
                                                        'wrist_2_joint',
                                                        'wrist_3_joint']
