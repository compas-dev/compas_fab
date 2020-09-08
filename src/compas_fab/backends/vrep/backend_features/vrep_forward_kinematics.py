from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces import ForwardKinematics
from compas_fab.backends.vrep.helpers import vrep_pose_to_frame

__all__ = [
    'VrepForwardKinematics',
]


class VrepForwardKinematics(ForwardKinematics):
    """Callable to calculate forward kinematics to get the current end-effector pose.
    """
    def __init__(self, client):
        self.client = client

    def forward_kinematics(self, robot=None, configuration=None, group=None, options=None):
        """Calculates forward kinematics to get the current end-effector pose.

        Args:
            robot (:class:`compas_fab.robots.Robot`): The robot instance for which forward kinematics is being calculated.
            configuration (:obj:`None`): Unused parameter.
            group (:obj:`int`): Integer referencing the desired robot group.
            options (:obj:`dict`): Unused parameter.

        Examples:

            >>> from compas_fab.robots import *
            >>> from compas_fab.backends import VrepClient
            >>> with VrepClient() as client:
            ...     frame = client.forward_kinematics(robot=None, configuration=None, group=0)

        Returns:
            An instance of :class:`Frame`.
        """

        _res, _, pose, _, _ = self.client.run_child_script('getIkTipPose', [group], [], [])
        return vrep_pose_to_frame(pose, self.client.scale)
