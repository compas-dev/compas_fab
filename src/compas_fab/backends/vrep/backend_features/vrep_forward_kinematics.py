from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.vrep.helpers import assert_robot
from compas_fab.backends.vrep.helpers import vrep_pose_to_frame


class VrepForwardKinematics(object):
    def __init__(self, client):
        self.client = client
        thing = VrepClient

    def __call__(self, robot):
        return self.forward_kinematics(robot)

    def forward_kinematics(self, robot):
        """Calculates forward kinematics to get the current end-effector pose.

        Args:
            robot (:class:`compas_fab.robots.Robot`): Robot instance.

        Examples:

            >>> from compas_fab.robots import *
            >>> from compas_fab.backends import VrepClient
            >>> with VrepClient() as client:
            ...     frame = client.forward_kinematics(rfl.Robot('A'))

        Returns:
            An instance of :class:`Frame`.
        """
        assert_robot(robot)

        _res, _, pose, _, _ = self.client.run_child_script('getIkTipPose', [robot.model.attr['index']], [], [])
        return vrep_pose_to_frame(pose, self.client.scale)
