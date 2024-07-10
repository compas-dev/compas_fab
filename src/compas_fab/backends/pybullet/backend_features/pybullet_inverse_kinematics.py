from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import compas

if not compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from compas_robots import Configuration  # noqa: F401
        from compas.geometry import Frame  # noqa: F401
        from typing import Optional  # noqa: F401
        from typing import Iterator  # noqa: F401
        from typing import List  # noqa: F401
        from typing import Tuple  # noqa: F401
        from typing import Dict  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import FrameTarget  # noqa: F401
        from compas_fab.robots import AttachedCollisionMesh  # noqa: F401
        from compas_fab.backends import PyBulletClient  # noqa: F401


import math
import random

from compas_robots.model import Joint

from compas_fab.backends.exceptions import InverseKinematicsError
from compas_fab.backends.interfaces import InverseKinematics
from compas_fab.backends.pybullet.conversions import pose_from_frame
from compas_fab.utilities import LazyLoader
from compas_fab.utilities import from_tcf_to_t0cf

pybullet = LazyLoader("pybullet", globals(), "pybullet")


__all__ = [
    "PyBulletInverseKinematics",
]


class PyBulletInverseKinematics(InverseKinematics):
    """Callable to calculate the robot's inverse kinematics for a given frame."""

    def iter_inverse_kinematics(self, target, start_state=None, group=None, options=None):
        # type: (FrameTarget, Optional[RobotCellState], Optional[str], Optional[Dict]) -> Iterator[Tuple[List[float], List[str]]]
        """Calculate the robot's inverse kinematic for a given frame.

        Parameters
        ----------
        target: :class:`compas.geometry.FrameTarget`
            The Frame Target to calculate the inverse for.
        start_state : :class:`compas_fab.robots.RobotCellState`, optional
            The starting state to calculate the inverse kinematics for.
        group: str, optional
            The planning group used for determining the end effector and labeling
            the ``start_configuration``. Defaults to the robot's main planning group.
        options: dict, optional
            Dictionary containing the following key-value pairs:

            - ``"link_name"``: (:obj:`str`, optional ) Name of the link for which
              to compute the inverse kinematics.  Defaults to the given group's end
              effector.
            - ``"semi-constrained"``: (:obj:`bool`, optional) When ``True``, only the
              position of the target is considered. The orientation of frame will not be considered
              in the calculation.  Defaults to ``False``.
            - ``"enforce_joint_limits"``: (:obj:`bool`, optional) When ``False``, the
              robot's joint limits will be ignored in the calculation.  Defaults to
              ``True``.
            - ``"high_accuracy"``:  (:obj:`bool`, optional) When ``True``, the
              solver will iteratively try to approach minimum deviation from the requested
              target frame.  Defaults to ``True``.
            - ``"high_accuracy_threshold"``:  (:obj:`float`, optional) Defines the maximum
              acceptable distance threshold for the high accuracy solver. Defaults to ``1e-4``.
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

        client = self.client  # type: PyBulletClient # Trick to keep intellisense happy
        robot = client.robot

        high_accuracy = options.get("high_accuracy", True)
        max_results = options.get("max_results", 100)
        link_name = options.get("link_name") or robot.get_end_effector_link_name(group)

        cached_robot_model = client.get_cached_robot_model(robot)
        body_id = client.get_uid(cached_robot_model)
        link_id = client._get_link_id_by_name(link_name, cached_robot_model)

        # Target frame
        target_frame = target.target_frame
        if robot.need_scaling:
            target_frame = target_frame.scaled(1.0 / robot.scale_factor)
            # Now target_frame is back in meter scale

        # Tool Coordinate Frame if there are tools attached
        if self.robot_cell:
            attached_tool = self.robot_cell.get_attached_tool(start_state, group)
            if attached_tool:
                target_frame = from_tcf_to_t0cf(target_frame, attached_tool.frame)
                # Attached tool frames does not need scaling because Tools are modelled in meter scale

        point, orientation = pose_from_frame(target_frame)

        joints = cached_robot_model.get_configurable_joints()
        joints.sort(key=lambda j: j.attr["pybullet"]["id"])
        joint_names = [joint.name for joint in joints]

        lower_limits = [joint.limit.lower if joint.type != Joint.CONTINUOUS else 0 for joint in joints]
        upper_limits = [joint.limit.upper if joint.type != Joint.CONTINUOUS else 2 * math.pi for joint in joints]

        start_configuration = start_state.robot_configuration or robot.zero_configuration(group)
        start_configuration = client.set_robot_configuration(start_configuration, group)

        # Rest pose is PyBullet's way of defining the initial guess for the IK solver
        # The order of the values needs to match with pybullet's joint id order
        rest_poses = self._get_rest_poses(joint_names, start_configuration)

        # Prepare Parameters for calling pybullet.calculateInverseKinematics
        ik_options = dict(
            bodyUniqueId=body_id,
            endEffectorLinkIndex=link_id,
            targetPosition=point,
            physicsClientId=client.client_id,
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
        if high_accuracy:
            ik_options.update(
                dict(
                    joints=joints,
                    threshold=options.get("high_accuracy_threshold", 1e-4),
                    max_iter=options.get("high_accuracy_max_iter", 20),
                )
            )
        # Loop to get multiple results
        for _ in range(max_results):

            # Ready to call IK
            if high_accuracy:
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

            # if not joint_positions:
            #     raise InverseKinematicsError()

            # Normally, no results ends the iteration, but if we have no solution
            # because the accurate IK solving says it's not close enough, we can retry
            # with a new randomized seed
            if close_enough:
                yield joint_positions, joint_names

            # Randomize joints to get a different solution on the next iter
            client._set_joint_positions(
                [joint.attr["pybullet"]["id"] for joint in joints],
                [random.uniform(*limits) for limits in zip(lower_limits, upper_limits)],
                body_id,
            )

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
            # print("Iter: %d, Distance: %s" % (iter, distance_squared))
            close_enough = distance_squared < threshold * threshold
            kwargs["restPoses"] = joint_poses
            iter += 1

        return joint_poses, close_enough

    def _get_rest_poses(self, joint_names, configuration):
        name_value_map = {
            configuration.joint_names[i]: configuration.joint_values[i] for i in range(len(configuration.joint_names))
        }
        return [name_value_map[name] for name in joint_names]
