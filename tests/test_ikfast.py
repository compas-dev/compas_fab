from __future__ import print_function
import os
import time
import pytest
from numpy.testing import assert_array_almost_equal

from compas.geometry import Frame
from compas.geometry import Translation
from compas.datastructures import Mesh
from compas.robots import LocalPackageMeshLoader
from compas.robots import RobotModel

import compas_fab
from compas_fab.backends import RosClient
from compas_fab.robots import Robot as RobotClass
from compas_fab.robots import RobotSemantics
from compas_fab.robots import PlanningScene
from compas_fab.robots import CollisionMesh
from compas_fab.robots.ur5 import Robot

from compas_fab.backends.pybullet import attach_end_effector_geometry, \
convert_mesh_to_pybullet_body, get_TCP_pose, create_pb_robot_from_ros_urdf

from compas_fab.backends.ros.plugins_choreo import get_ik_tool_link_pose

from conrob_pybullet import load_pybullet, connect, disconnect, wait_for_user, \
    LockRenderer, has_gui, get_model_info, get_pose, euler_from_quat, draw_pose, \
    get_link_pose, link_from_name, create_attachment, add_fixed_constraint, \
    create_obj, set_pose, get_sample_fn, violates_limits, joints_from_names, \
    set_joint_positions

try:
    import ikfast_ur5
except ImportError as e:
    assert False, '\x1b[6;30;43m' + '{}, please install ikfast_pybind'.format(e) + '\x1b[0m'


@pytest.mark.wip
def test_ikfast_forward_kinematics():
    """TODO: this test_function can by pybullet-free"""
    urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
    srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')
    urdf_pkg_name = 'ur_description'

    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
    robot = RobotClass(model, semantics=semantics)

    base_link_name = robot.get_base_link_name()
    ik_joint_names = robot.get_configurable_joint_names()
    ik_tool_link_name = robot.get_end_effector_link_name()

    connect(use_gui=False)
    pb_robot = create_pb_robot_from_ros_urdf(urdf_filename, urdf_pkg_name)
    pb_ik_joints = joints_from_names(pb_robot, ik_joint_names)

    max_attempts = 20
    sample_fn = get_sample_fn(pb_robot, pb_ik_joints)

    for _ in range(max_attempts):
        # randomly sample within joint limits
        conf = sample_fn()
        # sanity joint limit violation check
        assert not violates_limits(pb_robot, pb_ik_joints, conf)

        # ikfast's FK
        fk_fn = ikfast_ur5.get_fk
        ikfast_FK_pb_pose = get_ik_tool_link_pose(fk_fn, pb_robot, ik_joint_names, base_link_name, conf)
        # print('ikfast FK sol: {}'.format(ikfast_FK_pb_pose))

        # pybullet's FK
        set_joint_positions(pb_robot, pb_ik_joints, conf)
        TCP_pb_pose = get_TCP_pose(pb_robot, ik_tool_link_name, return_pb_pose=True)
        # print('pybullet FK sol: {}'.format(TCP_pb_pose))

        assert_array_almost_equal(TCP_pb_pose[0], ikfast_FK_pb_pose[0])
        assert_array_almost_equal(TCP_pb_pose[1], ikfast_FK_pb_pose[1])

    # if has_gui():
    #     draw_pose(ikfast_FK_pb_pose, length=0.04)
    #     draw_pose(TCP_pb_pose, length=0.04)
    #     wait_for_user()
