import pytest

import compas_fab
from compas_fab.backends import PybulletClient
from compas_fab.backends.pybullet import pb_pose_from_Frame

from compas.geometry import Frame
from compas_fab.backends import RosClient
from compas_fab.robots.ur5 import Robot
import ikfast_ur5

from pybullet_planning import wait_for_user, has_gui, draw_pose
from numpy.testing import assert_almost_equal

@pytest.fixture
def n_attempts():
    # number of checking attempts, used in ik/fk
    return 100

@pytest.fixture
def robot_data():
    urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
    srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')
    return urdf_filename, srdf_filename

def test_connection(viewer):
    with PybulletClient(viewer=viewer) as client:
        print('Connected: {}'.format(client.is_connected))

# @pytest.mark.wip
def test_fk_ik(viewer, n_attempts, robot_data):
    urdf_filename, srdf_filename = robot_data

    with PybulletClient(viewer=viewer) as client:
        robot = Robot(client)
        pb_robot = client.create_pb_robot(urdf_filename)

        for _ in range(n_attempts):
            conf = robot.random_configuration()
            frame_RCF = robot.forward_kinematics(conf, pb_robot=pb_robot)
            # draw_pose(pb_pose_from_Frame(frame_RCF))
            # wait_for_user()
            try:
                ik_conf = robot.inverse_kinematics(frame_RCF, ee_link=robot.get_end_effector_link_name(), pb_robot=pb_robot)
            except TypeError:
                # when no ik solution is found, ik will return None, which causes `_scale_joint_values` will raise:
                # TypeError: zip argument #1 must support iteration
                continue
            # check agreement between fk and ik solutions
            assert_almost_equal(ik_conf.values, conf.values, decimal=6)
