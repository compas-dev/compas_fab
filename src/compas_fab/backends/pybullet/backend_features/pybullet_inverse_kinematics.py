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
        from typing import Generator  # noqa: F401

        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import Target  # noqa: F401
        from compas_fab.robots import Robot  # noqa: F401
        from compas_fab.backends import PyBulletClient  # noqa: F401
        from compas_fab.backends import PyBulletPlanner  # noqa: F401


import math
import random

from compas_robots.model import Joint

from compas_fab.backends.exceptions import InverseKinematicsError
from compas_fab.backends.interfaces import InverseKinematics
from compas_fab.backends.pybullet.conversions import pose_from_frame
from compas_fab.utilities import LazyLoader
from compas_fab.utilities import from_tcf_to_t0cf

from compas_fab.robots import FrameTarget

pybullet = LazyLoader("pybullet", globals(), "pybullet")


__all__ = [
    "PyBulletInverseKinematics",
]


class PyBulletInverseKinematics(InverseKinematics):
    """Mix-in functions to calculate the robot's inverse kinematics for a given target."""

    def iter_inverse_kinematics(self, target, robot_cell_state=None, group=None, options=None):
        # type: (Target, Optional[RobotCellState], Optional[str], Optional[Dict]) -> Generator[Tuple[List[float], List[str]], None, None]

        if isinstance(target, FrameTarget):
            return self.iter_inverse_kinematics_frame_target(target, robot_cell_state, group, options)
        else:
            raise NotImplementedError("{} is not supported by PyBulletInverseKinematics".format(type(target)))

    def iter_inverse_kinematics_frame_target(self, target, robot_cell_state=None, group=None, options=None):
        # type: (FrameTarget, Optional[RobotCellState], Optional[str], Optional[Dict]) -> Generator[Tuple[List[float], List[str]], None, None]
        """Calculate the robot's inverse kinematic for a given frame.

        Parameters
        ----------
        target: :class:`compas.geometry.FrameTarget`
            The Frame Target to calculate the inverse for.
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`, optional
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

        # Housekeeping for intellisense
        planner = self  # type: PyBulletPlanner
        client = planner.client  # type: PyBulletClient
        robot = client.robot  # type: Robot
        group = group or robot.main_group_name

        high_accuracy = options.get("high_accuracy", True)
        max_results = options.get("max_results", 100)
        link_name = options.get("link_name") or robot.get_end_effector_link_name(group)

        # Setting the entire robot cell state, including the robot configuration
        robot_cell_state = robot_cell_state.copy()  # Make a copy to avoid modifying the original
        planner.set_robot_cell_state(robot_cell_state)

        body_id = client.robot_puid
        link_id = client.robot_link_puids[link_name]

        # Target frame
        target = self._scale_input_target(target)
        target_frame = target.target_frame

        # Tool Coordinate Frame if there are tools attached
        attached_tool_id = robot_cell_state.get_attached_tool_id(group)
        if attached_tool_id:
            target_frame = planner.from_tcf_to_t0cf([target_frame], attached_tool_id)[0]

        point, orientation = pose_from_frame(target_frame)

        # Get list of keys (joint_name) from the joint_ids dict in the order of its values (puid)
        joint_names_sorted, joint_ids_sorted = client.get_configurable_joint_names_and_puid()

        # Get joint limits in the same order as the joint_ids
        lower_limits = []
        upper_limits = []
        for joint_name in joint_names_sorted:
            joint = robot.get_joint_by_name(joint_name)
            lower_limits.append(joint.limit.lower if joint.type != Joint.CONTINUOUS else 0)
            upper_limits.append(joint.limit.upper if joint.type != Joint.CONTINUOUS else 2 * math.pi)

        # Rest pose is PyBullet's way of defining the initial guess for the IK solver
        start_configuration = robot_cell_state.robot_configuration or robot.zero_configuration(group)

        # The order of the values needs to match with pybullet's joint id order
        rest_poses = [start_configuration[joint_name] for joint_name in joint_names_sorted]

        # Prepare Parameters for calling pybullet.calculateInverseKinematics
        ik_options = dict(
            bodyUniqueId=body_id,
            endEffectorLinkIndex=link_id,
            targetPosition=point,
            targetOrientation=orientation,
            physicsClientId=client.client_id,
        )

        # Options for enforce joint limits mode
        if options.get("enforce_joint_limits", True):
            # I don't know what jointRanges needs to be.  Erwin Coumans knows, but he isn't telling.
            # https://stackoverflow.com/questions/49674179/understanding-inverse-kinematics-pybullet
            # https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/preview?pru=AAABc7276PI*zazLer2rlZ8tAUI8lF98Kw#heading=h.9i02ojf4k3ve
            joint_ranges = [u - l for u, l in zip(upper_limits, lower_limits)]
            ik_options.update(
                dict(
                    lowerLimits=lower_limits,
                    upperLimits=upper_limits,
                    jointRanges=joint_ranges,
                    restPoses=rest_poses,
                )
            )
        # Options for semi-constrained mode, skipping the targetOrientation
        if options.get("semi-constrained"):
            ik_options.pop("targetOrientation")

        # Options for high accuracy mode
        if high_accuracy:
            ik_options.update(
                dict(
                    joint_ids_sorted=joint_ids_sorted,
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
                # Setting the robot configuration so we can perform collision checking
                # This also updates the robot's pose in the client
                for [joint_name, joint_position] in zip(joint_names_sorted, joint_positions):
                    robot_cell_state.robot_configuration[joint_name] = joint_position
                planner.set_robot_cell_state(robot_cell_state)
                # Collision checking
                # TODO: Implement collision checking
                yield joint_positions, joint_names_sorted

            # Randomize joint values to get a different solution on the next iter
            for lower_limit, upper_limit, joint_id in zip(lower_limits, upper_limits, joint_ids_sorted):
                random_value = random.uniform(lower_limit, upper_limit)
                client._set_joint_position(joint_id, random_value, client.robot_puid)

    def _accurate_inverse_kinematics(self, joint_ids_sorted, threshold, max_iter, **kwargs):
        # Based on these examples
        # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_kinematics_husky_kuka.py#L81
        close_enough = False
        iter = 0
        body_id = kwargs["bodyUniqueId"]
        link_id = kwargs["endEffectorLinkIndex"]
        target_position = kwargs["targetPosition"]

        while not close_enough and iter < max_iter:
            joint_poses = pybullet.calculateInverseKinematics(**kwargs)
            for i in range(len(joint_ids_sorted)):
                pybullet.resetJointState(body_id, joint_ids_sorted[i], joint_poses[i])

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
