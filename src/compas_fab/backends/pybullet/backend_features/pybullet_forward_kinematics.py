from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces import ForwardKinematics


class PyBulletForwardKinematics(ForwardKinematics):
    """Callable to calculate the robot's forward kinematic."""
    def __init__(self, client):
        self.client = client

    def forward_kinematics(self, configuration, group=None, options=None):
        # """Calculate the robot's forward kinematic.
        #
        # Parameters
        # ----------
        # configuration : :class:`compas_fab.robots.Configuration`
        #     The full configuration to calculate the forward kinematic for. If no
        #     full configuration is passed, the zero-joint state for the other
        #     configurable joints is assumed.
        # group : str, optional
        #     Unused parameter.
        # options : dict, optional
        #     Dictionary containing the following key-value pairs:
        #
        #     - ``"base_link"``: (:obj:`str`) The name of the base link.
        #     - ``"ee_link"``: (:obj:`str`, optional) The name of the link to
        #       calculate the forward kinematics for. Defaults to the group's end
        #       effector link.
        #
        # Returns
        # -------
        # :class:`Frame`
        #     The frame in the world's coordinate system (WCF).
        # """
        raise NotImplementedError
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
