from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.backend_feature_interfaces import ForwardKinematics


class PyBulletForwardKinematics(ForwardKinematics):
    def __init__(self, client):
        self.client = client

    def forward_kinematics(self, configuration, group=None, options={}):
        pass
        # """
        # """
        # ee_link_name = options['ee_link_name']
        # joints = self.client.joints_from_names(self.client.robot_uid, configuration.joint_names)
        # ee_link = self.client.link_from_name(self.client.robot_uid, ee_link_name)
        # self.client.set_joint_positions(self.client.robot_uid, joints, configuration.values)
        # pose = self.client.get_link_pose(self.client.robot_uid, ee_link)
        # if options.get('check_collision'):
        #     collision, names = self.client.client.collision_check()
        #     if collision:
        #         raise CollisionError(*names)
        # return frame_from_pose(pose)
