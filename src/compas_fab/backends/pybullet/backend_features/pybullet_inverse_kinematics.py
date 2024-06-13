from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import math
import random

from compas_robots.model import Joint

from compas_fab.backends.exceptions import InverseKinematicsError
from compas_fab.backends.interfaces import InverseKinematics
from compas_fab.backends.pybullet.conversions import pose_from_frame
from compas_fab.utilities import LazyLoader

pybullet = LazyLoader("pybullet", globals(), "pybullet")


__all__ = [
    "PyBulletInverseKinematics",
]


class PyBulletInverseKinematics(InverseKinematics):
    """Callable to calculate the robot's inverse kinematics for a given frame."""

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
            The planning group used for determining the end effector and labeling
            the ``start_configuration``. Defaults to the robot's main planning group.
        options: dict, optional
            Dictionary containing the following key-value pairs:

            - ``"link_name"``: (:obj:`str`, optional ) Name of the link for which
              to compute the inverse kinematics.  Defaults to the given group's end
              effector.
            - ``"semi-constrained"``: (:obj:`bool`, optional) When ``True``, only the
              position and not the orientation of ``frame_WCF`` will not be considered
              in the calculation.  Defaults to ``False``.
            - ``"enforce_joint_limits"``: (:obj:`bool`, optional) When ``False``, the
              robot's joint limits will be ignored in the calculation.  Defaults to
              ``True``.
            - ``"high_accuracy"``:  (:obj:`bool`, optional) When ``True``, the
              solver will iteratively try to approach minimum deviation from the requested
              target frame.  Defaults to ``True``.
            - ``"high_accuracy_threshold"``:  (:obj:`float`, optional) Defines the maximum
              acceptable distance threshold for the high accuracy solver. Defaults to ``1e-6``.
            - ``"high_accuracy_max_iter"``:  (:obj:`float`, optional) Defines the maximum
              number of iterations to use for the high accuracy solver. Defaults to ``20``.
            - ``"max_results"``: (:obj:`int`) Maximum number of results to return.
              Defaults to ``100``.

        Yields
        ------
        :obj:`tuple` of :obj:`list`
            A tuple of 2 elements containing a list of joint positions and a list of matching joint names.

        Raises
        ------
        :class:`compas_fab.backends.InverseKinematicsError`
        """
        options = options or {}
        high_accuracy = options.get("high_accuracy", True)
        max_results = options.get("max_results", 100)
        link_name = options.get("link_name") or robot.get_end_effector_link_name(group)
        cached_robot_model = self.client.get_cached_robot_model(robot)
        body_id = self.client.get_uid(cached_robot_model)
        link_id = self.client._get_link_id_by_name(link_name, cached_robot_model)
        point, orientation = pose_from_frame(frame_WCF)

        joints = cached_robot_model.get_configurable_joints()
        joints.sort(key=lambda j: j.attr["pybullet"]["id"])
        joint_names = [joint.name for joint in joints]

        lower_limits = [joint.limit.lower if joint.type != Joint.CONTINUOUS else 0 for joint in joints]
        upper_limits = [joint.limit.upper if joint.type != Joint.CONTINUOUS else 2 * math.pi for joint in joints]

        start_configuration = start_configuration or robot.zero_configuration(group)
        start_configuration = self.client.set_robot_configuration(robot, start_configuration, group)

        rest_poses = self._get_rest_poses(joint_names, start_configuration)

        for _ in range(max_results):
            ik_options = dict(
                bodyUniqueId=body_id,
                endEffectorLinkIndex=link_id,
                targetPosition=point,
                physicsClientId=self.client.client_id,
            )

            if options.get("enforce_joint_limits", True):
                # I don't know what jointRanges needs to be.  Erwin Coumans knows, but he isn't telling.
                # https://stackoverflow.com/questions/49674179/understanding-inverse-kinematics-pybullet
                # https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/preview?pru=AAABc7276PI*zazLer2rlZ8tAUI8lF98Kw#heading=h.9i02ojf4k3ve
                joint_ranges = [u - l for u, l in zip(upper_limits, lower_limits)]

                if options.get("semi-constrained"):
                    ik_options.update(
                        dict(
                            lowerLimits=lower_limits,
                            upperLimits=upper_limits,
                            jointRanges=joint_ranges,
                            restPoses=rest_poses,
                        )
                    )
                else:
                    ik_options.update(
                        dict(
                            targetPosition=point,
                            targetOrientation=orientation,
                            lowerLimits=lower_limits,
                            upperLimits=upper_limits,
                            jointRanges=joint_ranges,
                            restPoses=rest_poses,
                        )
                    )
            else:
                if not options.get("semi-constrained"):
                    ik_options.update(
                        dict(
                            targetOrientation=orientation,
                        )
                    )

            # Ready to call IK
            if high_accuracy:
                ik_options.update(
                    dict(
                        joints=joints,
                        threshold=options.get("high_accuracy_threshold", 1e-4),
                        max_iter=options.get("high_accuracy_max_iter", 20),
                    )
                )
                joint_positions, close_enough = self._accurate_inverse_kinematics(**ik_options)

                # NOTE: In principle, this accurate iter IK should work out of the
                # pybullet box, but the results seem to be way off target. For now,
                # I'm leaving the legacy iterative accurate ik in python as per
                # older examples of pybullet, until we figure out why the builtin
                # one is not cooperating.

                # ik_options.update(dict(
                #     residualThreshold=options.get('high_accuracy_threshold', 1e-6),
                #     maxNumIterations=options.get('high_accuracy_max_iter', 20),
                # ))
                # joint_positions = pybullet.calculateInverseKinematics(**ik_options)
            else:
                joint_positions = pybullet.calculateInverseKinematics(**ik_options)
                close_enough = True  # yeah, let's say we trust it

            if not joint_positions:
                raise InverseKinematicsError()

            # Randomize joints to get a different solution on the next iter
            self.client._set_joint_positions(
                [joint.attr["pybullet"]["id"] for joint in joints],
                [random.uniform(*limits) for limits in zip(lower_limits, upper_limits)],
                body_id,
            )

            # Normally, no results ends the iteration, but if we have no solution
            # because the accurate IK solving says it's not close enough, we can retry
            # with a new randomized seed
            if not close_enough and high_accuracy:
                continue

            rest_poses = joint_positions
            yield joint_positions, joint_names

    def _accurate_inverse_kinematics(self, joints, threshold, max_iter, **kwargs):
        # Based on these examples
        # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_kinematics_husky_kuka.py#L81
        close_enough = False
        iter = 0
        joint_ids = [joint.attr["pybullet"]["id"] for joint in joints]
        body_id = kwargs["bodyUniqueId"]
        link_id = kwargs["endEffectorLinkIndex"]
        target_position = kwargs["targetPosition"]

        while not close_enough and iter < max_iter:
            joint_poses = pybullet.calculateInverseKinematics(**kwargs)
            for i in range(len(joint_ids)):
                pybullet.resetJointState(body_id, joint_ids[i], joint_poses[i])

            link_state = pybullet.getLinkState(body_id, link_id)
            new_pose = link_state[4]

            diff = [
                target_position[0] - new_pose[0],
                target_position[1] - new_pose[1],
                target_position[2] - new_pose[2],
            ]
            # The distance is squared to avoid a sqrt operation
            distance_squared = diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]
            # Therefor, the threshold is squared as well
            close_enough = distance_squared < threshold * threshold
            kwargs["restPoses"] = joint_poses
            iter += 1

        return joint_poses, close_enough

    def _get_rest_poses(self, joint_names, configuration):
        name_value_map = {
            configuration.joint_names[i]: configuration.joint_values[i] for i in range(len(configuration.joint_names))
        }
        return [name_value_map[name] for name in joint_names]
