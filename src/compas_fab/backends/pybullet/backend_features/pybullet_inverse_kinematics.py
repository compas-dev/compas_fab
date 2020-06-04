from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.backend_feature_interfaces import InverseKinematics

__all__ = [
    'PyBulletInverseKinematics',
]


class PyBulletInverseKinematics(InverseKinematics):
    def __init__(self, client):
        self.client = client

    def inverse_kinematics(self, frame_WCF, start_configuration=None, group=None, options={}):
        pass
        # """
        # """
        # self.ensure_robot()
        # ee_link_name = options.get('ee_link_name')
        # pb_ee_link = self.client.link_from_name(self.client.robot_uid, ee_link_name)
        # joint_names = [name.decode('UTF-8') for name in self.client.get_joint_names(self.robot_uid, self.client.get_movable_joints(self.client.robot_uid))]
        # joint_positions = inverse_kinematics(self.client.robot_uid, pb_ee_link, self.client.pose_from_frame(frame), max_iterations=attempts)
        # if not joint_positions:
        #     raise InverseKinematicsError()
        # return joint_positions, joint_names
