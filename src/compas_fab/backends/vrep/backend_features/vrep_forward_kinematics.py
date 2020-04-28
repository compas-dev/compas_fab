from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.backend_feature_interfaces import ForwardKinematics
from compas_fab.backends.vrep.helpers import vrep_pose_to_frame


class VrepForwardKinematics(ForwardKinematics):
    def __init__(self, client):
        self.client = client

    def forward_kinematics(self, configuration, group=None, options={}):  # !!! must find all calls to this and adapt !!!
        return self.forward_kinematics_deprecated(group)

    def forward_kinematics_deprecated(self, group):
        """Calculates forward kinematics to get the current end-effector pose.

        Args:
            group (int): Integer referencing the desired robot group.

        Examples:

            >>> from compas_fab.robots import *
            >>> from compas_fab.backends import VrepClient
            >>> with VrepClient() as client:
            ...     frame = client.forward_kinematics(None, 0)

        Returns:
            An instance of :class:`Frame`.
        """

        _res, _, pose, _, _ = self.client.run_child_script('getIkTipPose', [group], [], [])
        return vrep_pose_to_frame(pose, self.client.scale)
