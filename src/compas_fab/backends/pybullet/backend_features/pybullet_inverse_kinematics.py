from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces import InverseKinematics

__all__ = [
    'PyBulletInverseKinematics',
]


class PyBulletInverseKinematics(InverseKinematics):
    """Callable to calculate the robot's inverse kinematics for a given frame."""
    def __init__(self, client):
        self.client = client

    def inverse_kinematics(self, frame_WCF, start_configuration=None, group=None, options=None):
        # """Calculate the robot's inverse kinematic for a given frame.
        #
        # Parameters
        # ----------
        # frame_WCF: :class:`compas.geometry.Frame`
        #     The frame to calculate the inverse for.
        # start_configuration: :class:`compas_fab.robots.Configuration`, optional
        #     If passed, the inverse will be calculated such that the calculated
        #     joint positions differ the least from the start_configuration.
        #     Defaults to the init configuration.
        # group: str, optional
        #     The planning group used for calculation. Defaults to the robot's
        #     main planning group.
        # options: dict, optional
        #     Dictionary containing the following key-value pairs:
        #
        #     - ``"base_link"``: (:obj:`str`) Name of the base link.
        #     - ``"avoid_collisions"``: (:obj:`bool`, optional) Whether or not to avoid collisions.
        #       Defaults to `True`.
        #     - ``"constraints"``: (:obj:`list` of :class:`compas_fab.robots.Constraint`, optional)
        #       A set of constraints that the request must obey. Defaults to `None`.
        #     - ``"attempts"``: (:obj:`int`, optional) The maximum number of inverse kinematic attempts.
        #       Defaults to `8`.
        #     - ``"attached_collision_meshes"``: (:obj:`list` of :class:`compas_fab.robots.AttachedCollisionMesh`, optional)
        #       Defaults to `None`.
        #
        # Returns
        # -------
        # :class:`compas_fab.robots.Configuration`
        #     The planning group's configuration.
        # """
        pass
        # """
        # """
        # if options is None:
        #     options = {}
        # self.ensure_robot()
        # ee_link_name = options.get('ee_link_name')
        # pb_ee_link = self.client.link_from_name(self.client.robot_uid, ee_link_name)
        # joint_names = [name.decode('UTF-8') for name in self.client.get_joint_names(self.robot_uid, self.client.get_movable_joints(self.client.robot_uid))]
        # joint_positions = inverse_kinematics(self.client.robot_uid, pb_ee_link, self.client.pose_from_frame(frame), max_iterations=attempts)
        # if not joint_positions:
        #     raise InverseKinematicsError()
        # return joint_positions, joint_names
