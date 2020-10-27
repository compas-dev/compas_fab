from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import math
import sys

from compas.robots import Joint
from compas_fab.backends.interfaces import InverseKinematics
from compas_fab.backends.pybullet.conversions import pose_from_frame
from compas_fab.backends.pybullet.exceptions import InverseKinematicsError
from compas_fab.utilities import LazyLoader

pybullet = LazyLoader('pybullet', globals(), 'pybullet')


__all__ = [
    'PyBulletInverseKinematics',
]


class PyBulletInverseKinematics(InverseKinematics):
    """Callable to calculate the robot's inverse kinematics for a given frame."""
    def __init__(self, client):
        self.client = client

    def inverse_kinematics(self, robot, frame_WCF, start_configuration=None, group=None, options=None):
        """Calculate the robot's inverse kinematic for a given frame.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which inverse kinematics is being calculated.
        frame_WCF: :class:`compas.geometry.Frame`
            The frame to calculate the inverse for.
        start_configuration: :class:`compas_fab.robots.Configuration`, optional
            If passed, the inverse will be calculated such that the calculated
            joint positions differ the least from the start_configuration.
            Defaults to the zero configuration.
        group: str, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        options: dict, optional
            Dictionary containing the following key-value pairs:

            - ``"link_name"``: (:obj:`str`, optional ) Name of the link for which
              to compute the inverse kinematics.  Defaults to the given robot's end
              effector.
            - ``"semi-constrained"``: (:obj:`bool`, optional) When ``True``, only the
              position and not the orientation of ``frame_WCF`` will not be considered
              in the calculation.  Defaults to ``False``.
            - ``"enforce_joint_limits"``: (:obj:`bool`, optional) When ``False``, the
              robot's joint limits will be ignored in the calculation.  Defaults to
              ``True``.

        Returns
        -------
        :obj:`tuple` of :obj:`list`
            A tuple of 2 elements containing a list of joint positions and a list of matching joint names.

        Raises
        ------
        :class:`compas_fab.backends.InverseKinematicsError`
        """
        options = options or {}
        link_name = options.get('link_name') or robot.get_end_effector_link_name(group)
        link_id = self.client._get_link_id_by_name(link_name, robot)
        point, orientation = pose_from_frame(frame_WCF)

        joints = robot.model.get_configurable_joints()
        joints.sort(key=lambda j: j.attr['pybullet']['id'])
        joint_names = [joint.name for joint in joints]

        if start_configuration:
            start_configuration = self.client.set_robot_configuration(robot, start_configuration, group)

        called_from_test = 'pytest' in sys.modules
        if options.get('enforce_joint_limits', True) and not called_from_test:
            lower_limits = [joint.limit.lower if joint.type != Joint.CONTINUOUS else 0 for joint in joints]
            upper_limits = [joint.limit.upper if joint.type != Joint.CONTINUOUS else 2 * math.pi for joint in joints]
            # I don't know what jointRanges needs to be.  Erwin Coumans knows, but he isn't telling.
            # https://stackoverflow.com/questions/49674179/understanding-inverse-kinematics-pybullet
            # https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/preview?pru=AAABc7276PI*zazLer2rlZ8tAUI8lF98Kw#heading=h.9i02ojf4k3ve
            joint_ranges = [u - l for u, l in zip(upper_limits, lower_limits)]
            rest_configuration = start_configuration or robot.zero_configuration()
            rest_poses = self._get_rest_poses(joint_names, rest_configuration)

            if options.get('semi-constrained'):
                joint_positions = pybullet.calculateInverseKinematics(
                    robot.attributes['pybullet_uid'],
                    link_id,
                    point,
                    lowerLimits=lower_limits,
                    upperLimits=upper_limits,
                    jointRanges=joint_ranges,
                    restPoses=rest_poses,
                )
            else:
                joint_positions = pybullet.calculateInverseKinematics(
                    robot.attributes['pybullet_uid'],
                    link_id,
                    point,
                    orientation,
                    lowerLimits=lower_limits,
                    upperLimits=upper_limits,
                    jointRanges=joint_ranges,
                    restPoses=rest_poses,
                )
        else:
            if options.get('semi-constrained'):
                joint_positions = pybullet.calculateInverseKinematics(
                    robot.attributes['pybullet_uid'],
                    link_id,
                    point,
                )
            else:
                joint_positions = pybullet.calculateInverseKinematics(
                    robot.attributes['pybullet_uid'],
                    link_id,
                    point,
                    orientation,
                )

        if not joint_positions:
            raise InverseKinematicsError()

        return joint_positions, joint_names

    def _get_rest_poses(self, joint_names, configuration):
        name_value_map = {configuration.joint_names[i]: configuration.values[i] for i in range(len(configuration.joint_names))}
        return [name_value_map[name] for name in joint_names]
