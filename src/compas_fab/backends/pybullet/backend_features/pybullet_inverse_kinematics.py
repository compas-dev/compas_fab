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
        from typing import Generator  # noqa: F401
        from typing import List  # noqa: F401
        from typing import Tuple  # noqa: F401
        from typing import Dict  # noqa: F401

        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import Target  # noqa: F401
        from compas_fab.robots import Robot  # noqa: F401
        from compas_fab.backends import PyBulletClient  # noqa: F401
        from compas_fab.backends import PyBulletPlanner  # noqa: F401


import math
import random

from compas.tolerance import TOL
from compas_robots.model import Joint

from compas_fab.backends.exceptions import InverseKinematicsError
from compas_fab.backends.exceptions import CollisionCheckError
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

    def inverse_kinematics(self, target, robot_cell_state=None, group=None, options=None):
        # type: (FrameTarget, Optional[RobotCellState], Optional[str], Optional[Dict]) -> Configuration
        """Calculate the robot's inverse kinematic for a given frame.

        The actual implementation can be found in the :meth:`iter_inverse_kinematics` method.
        Calling `inverse_kinematics()` will return the first solution found by the iterator,
        subsequent calls will return the next solution from the iterator. Once
        all solutions have been exhausted, the iterator will be re-initialized.

        Pybullet's inverse kinematics solver accepts FrameTarget and PointAxisTarget as input.
        The planner is a gradient descent solver, the initial position of the robot
        (supplied in the robot_cell_state) affects the first search attempt.
        Subsequent attempts will start from a random configuration, so the results may vary.

        For target-specific implementation details, see
        :meth:`iter_inverse_kinematics_frame_target` for
        :class:`compas_fab.robots.FrameTarget` and
        :meth:`iter_inverse_kinematics_point_axis_target` for
        :class:`compas_fab.robots.PointAxisTarget`.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which inverse kinematics is being calculated.
        target : :class:`compas_fab.robots.FrameTarget` or :class:`compas_fab.robots.PointAxisTarget`
            The target to calculate the inverse kinematics for.
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`, optional
            The starting state to calculate the inverse kinematics for.
            The robot's configuration in the scene is taken as the starting configuration.
        group : str, optional
            The planning group used for calculation.
        options : dict, optional
            Dictionary containing kwargs for arguments specific to
            the underlying function being called.
            See the target-specific function's documentation for details.

        Raises
        ------
        :class: `compas_fab.backends.exceptions.InverseKinematicsError`
            If no configuration can be found.

        Returns
        -------
        :obj:`compas_robots.Configuration`
            The calculated configuration.

        """
        # The caching mechanism is implemented in the iter_inverse_kinematics method
        # located in InverseKinematics class. This method is just a wrapper around it
        # so that Intellisense and Docs can point here.

        return super(PyBulletInverseKinematics, self).inverse_kinematics(target, robot_cell_state, group, options)

    def iter_inverse_kinematics(self, target, robot_cell_state=None, group=None, options=None):
        # type: (Target, Optional[RobotCellState], Optional[str], Optional[Dict]) -> Generator[Configuration | None]

        if isinstance(target, FrameTarget):
            return self.iter_inverse_kinematics_frame_target(target, robot_cell_state, group, options)
        else:
            raise NotImplementedError("{} is not supported by PyBulletInverseKinematics".format(type(target)))

    def iter_inverse_kinematics_frame_target(self, target, robot_cell_state=None, group=None, options=None):
        # type: (FrameTarget, Optional[RobotCellState], Optional[str], Optional[Dict]) -> Generator[Configuration | None]
        """Calculate the robot's inverse kinematic for a given FrameTarget.

        The PyBullet inverse kinematics solver make use of the gradient descent IK solver
        implemented in PyBullet. The solver is a gradient descent solver, so the initial
        position of the robot is important.

        This particular function wraps the PyBullet IK solver to provide a generator
        that can yield multiple IK solutions. The solver will make multiple attempts
        to find a solution. The first attempt will start from the robot's current configuration
        provided in the ``robot_cell_state``. The subsequent attempts will start from a random
        configuration, so the results may vary.

        Notes
        -----
        The number of attempts is determined by the ``max_results`` option.

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

            - ``"semi-constrained"``: (:obj:`bool`, optional) When ``True``, only the
              position of the target is considered. The orientation of frame will not be considered
              in the calculation.  Defaults to ``False``.
            - ``"high_accuracy"``:  (:obj:`bool`, optional) When ``True``, the
              solver will iteratively try to reach the ``high_accuracy_threshold``.
              Failure to reach the threshold within ``high_accuracy_max_iter`` will raise an exception.
              When ``False``, the solver will use the default pybullet solver, which does not
              guarantee any accuracy.
              Defaults to ``True``.
            - ``"high_accuracy_threshold"``:  (:obj:`float`, optional) Defines the maximum
              acceptable distance threshold for the high accuracy mode. Defaults to ``1e-4``.
            - ``"high_accuracy_max_iter"``:  (:obj:`float`, optional) Defines the maximum
              number of iterations to use for the high accuracy mode. Defaults to ``20``.
            - ``"max_results"``: (:obj:`int`) Maximum number of results to return.
              If set to 1, the solver will be deterministic, descending from the initial
              robot configuration.
              Defaults to ``100``.
            - ``"solution_uniqueness_threshold_prismatic"``: (:obj:`float`, optional) The minimum
              distance between two solutions in the prismatic joint space to consider them unique.
              Units are in meters. Defaults to ``3e-4``.
            - ``"solution_uniqueness_threshold_revolute"``: (:obj:`float`, optional) The minimum
              distance between two solutions in the revolute joint space to consider them unique.
              Units are in radians. Defaults to ``1e-3``.
            - ``"check_collision"``: (:obj:`bool`, optional)
              Whether or not to check for collision. Defaults to ``True``.
            - ``"return_full_configuration"``: (:obj:`bool`, optional)
                Whether or not to return the full configuration. Defaults to ``False``.

        Yields
        ------
        :obj:`tuple` of :obj:`list`
            A tuple of 2 elements containing a list of joint positions and a list of matching joint names.

        Raises
        ------
        :class:`compas_fab.backends.InverseKinematicsError`
            Indicates that no IK solution could be found by the kinematic solver
            after the maximum number of attempts (``max_results``). This can be caused by
            reachability or collision or both.

        :class:`compas_fab.backends.CollisionCheckError`
            If ``check_collision`` is enabled and the configuration is in collision.
            This is only raised if ``max_results`` is set to 1. In this case, the solver is
            deterministic (descending from the initial robot configuration) and this error
            indicates that the problem is caused by collision and not because of reachability.

        """
        options = options or {}

        # Housekeeping for intellisense
        planner = self  # type: PyBulletPlanner
        client = planner.client  # type: PyBulletClient
        robot = client.robot  # type: Robot
        group = group or robot.main_group_name

        high_accuracy = options.get("high_accuracy", True)
        max_results = options.get("max_results", 100)

        # Default options
        options["check_collision"] = options.get("check_collision", True)
        options["return_full_configuration"] = options.get("return_full_configuration", False)
        options["solution_uniqueness_threshold_prismatic"] = options.get(
            "solution_uniqueness_threshold_prismatic", 3e-4
        )
        options["solution_uniqueness_threshold_revolute"] = options.get("solution_uniqueness_threshold_revolute", 1e-3)

        # Setting the entire robot cell state, including the robot configuration
        robot_cell_state = robot_cell_state.copy()  # Make a copy to avoid modifying the original
        planner.set_robot_cell_state(robot_cell_state)

        # Pre-process target frame
        target = self._scale_input_target(target)
        target_frame = target.target_frame

        # TODO: Implement a fail fast mechanism to check if the attached tool and objects are in collision

        # Transform Tool Coordinate Frame if there are tools attached
        attached_tool_id = robot_cell_state.get_attached_tool_id(group)
        if attached_tool_id:
            target_frame = planner.from_tcf_to_pcf([target_frame], attached_tool_id)[0]

        # Formatting input for PyBullet
        body_id = client.robot_puid
        link_id = client.robot_link_puids[robot.get_end_effector_link_name(group)]
        point, orientation = pose_from_frame(target_frame)

        # Get list of joint_name and puids in order
        joint_names_and_puids = client.get_pose_joint_names_and_puids()
        joint_names_sorted = [joint_name for joint_name, _ in joint_names_and_puids]
        joint_ids_sorted = [joint_puid for _, joint_puid in joint_names_and_puids]
        joint_types_sorted = [robot.get_joint_by_name(joint_name).type for joint_name in joint_names_sorted]

        # Prepare `rest_poses` input
        # Rest pose is PyBullet's way of defining the initial guess for the IK solver
        # The order of the values needs to match with pybullet's joint id order
        # Start configuration needs to be a full_configuration, not negotiable.
        start_configuration = robot_cell_state.robot_configuration
        all_joint_names = robot.model.get_configurable_joint_names()
        assert set(all_joint_names) == set(start_configuration.keys()), "Robot configuration is missing some joints"
        rest_poses = client.build_pose_for_pybullet(start_configuration)
        rest_poses_dict = dict(zip(joint_names_sorted, rest_poses))

        # Prepare `lower_limits`` and `upper_limits` input
        # Get joint limits in the same order as the joint_ids
        lower_limits = []
        upper_limits = []
        for joint_name, joint_puid in joint_names_and_puids:
            # Check if the joint is in the planning group
            if joint_name not in robot.get_configurable_joint_names(group):
                lower_limits.append(rest_poses_dict[joint_name] - 0.01)
                upper_limits.append(rest_poses_dict[joint_name] + 0.01)
                continue
            joint = robot.get_joint_by_name(joint_name)
            lower_limits.append(joint.limit.lower if joint.type != Joint.CONTINUOUS else 0)
            upper_limits.append(joint.limit.upper if joint.type != Joint.CONTINUOUS else 2 * math.pi)

        # Prepare Parameters for calling pybullet.calculateInverseKinematics
        ik_options = dict(
            bodyUniqueId=body_id,
            endEffectorLinkIndex=link_id,
            targetPosition=point,
            targetOrientation=orientation,
            physicsClientId=client.client_id,
        )

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

        def set_random_config():
            # Function for setting random joint values for randomized search
            config = robot.random_configuration(group)
            client.set_robot_configuration(config)

        # Function to keep track of unique solutions
        solutions = []
        # Each joint has a different threshold for uniqueness
        uniqueness_thresholds_sorted = [
            (
                options["solution_uniqueness_threshold_prismatic"]
                if joint_type in [Joint.PRISMATIC, Joint.PLANAR]
                else options["solution_uniqueness_threshold_revolute"]
            )
            for joint_type in joint_types_sorted
        ]

        def solution_is_unique(joint_positions):
            """Check if the solution is unique by comparing with past solutions."""
            for past_solution in solutions:
                # Only if all joints are same, we consider the solution as not unique
                if all(
                    TOL.is_close(joint_position, past_joint_position, atol=threshold)
                    for joint_position, past_joint_position, threshold in zip(
                        joint_positions, past_solution, uniqueness_thresholds_sorted
                    )
                ):
                    return False
            return True

        # Loop to get multiple results
        for _ in range(max_results):

            # Calling the IK function (High accuracy or not)
            if high_accuracy:
                joint_positions, close_enough = self._accurate_inverse_kinematics(**ik_options)

                # NOTE: In principle, this accurate iter IK should work out of the
                # pybullet box, but the results seem to be way off target. For now,
                # I'm leaving the legacy iterative accurate ik in python as per
                # older examples of pybullet, until we figure out why the builtin
                # one is not cooperating.
                if not close_enough:
                    # If the solution is not close enough, we retry with a new randomized joint values
                    set_random_config()
                    continue
            else:
                joint_positions = pybullet.calculateInverseKinematics(**ik_options)
                # Without the high accuracy mode, there is no guarantee of accuracy

            assert len(joint_positions) == len(
                joint_names_and_puids
            ), "Number of returned joint positions from pybullet does not match number of joint ids"

            # Setting the robot configuration so we can perform collision checking
            # This also updates the robot's pose in the client
            # Pybullet may return mimic joints that are not in the robot's compas_fab configuration
            for [joint_name, joint_position] in zip(joint_names_sorted, joint_positions):
                if joint_name in robot_cell_state.robot_configuration:
                    robot_cell_state.robot_configuration[joint_name] = joint_position
            planner.set_robot_cell_state(robot_cell_state)

            # Collision checking
            if options.get("check_collision"):
                try:
                    planner.check_collision(robot_cell_state, options)
                except CollisionCheckError as e:
                    if options.get("verbose", False):
                        print("Collision detected. Skipping this solution.")
                        print(e)
                    # If max_results is 1, he user probably wants to know that the problem is caused by collision
                    # and not because there is no IK solution. So we re-raise the Collision Error
                    if max_results == 1:
                        raise e
                    # If there is more attempts, we skip this solution and try again with a new randomized joint values
                    set_random_config()
                    continue

            # Unique solution checking
            if not solution_is_unique(joint_positions):
                # If the solution is not unique, we retry with a new randomized joint values
                set_random_config()
                continue

            # If we got this far, we have a valid solution to yield
            solutions.append(joint_positions)
            return_full_configuration = options.get("return_full_configuration")
            configuration = self._build_configuration(
                joint_positions, joint_names_sorted, group, return_full_configuration, start_configuration
            )
            yield configuration

            # In order to generate multiple IK results,
            # we start the next loop iteration with randomized joint values
            set_random_config()

        # If no solution was found after max_results, raise an error
        if len(solutions) == 0:
            raise InverseKinematicsError(
                "No solution found after {} attempts (max_results).".format(max_results), target_pcf=target_frame
            )

    def _accurate_inverse_kinematics(self, joint_ids_sorted, threshold, max_iter, **kwargs):
        """Iterative inverse kinematics solver with a threshold for the distance to the target.

        This functions helps to get a more accurate solution by iterating over the IK solver
        until the distance to the target is below a certain threshold.
        This overcomes the limitations of the default pybullet solver, which does not guarantee any accuracy.

        Returns
        -------
        tuple of list of float, bool
            A tuple containing the joint positions and a boolean indicating if the solution is close enough to the target.
        """
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
