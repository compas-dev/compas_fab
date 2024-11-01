from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas import IPY

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:  # pragma: no cover
        from compas_robots import Configuration  # noqa: F401
        from typing import Optional  # noqa: F401
        from typing import Generator  # noqa: F401
        from typing import List  # noqa: F401
        from typing import Dict  # noqa: F401

        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import Target  # noqa: F401
        from compas_fab.robots import RobotCell  # noqa: F401
        from compas_fab.backends import PyBulletClient  # noqa: F401
        from compas_fab.backends import PyBulletPlanner  # noqa: F401

import math
from copy import deepcopy

from compas.geometry import Frame
from compas.geometry import Quaternion
from compas.geometry import axis_angle_from_quaternion
from compas.geometry import is_parallel_vector_vector
from compas.tolerance import TOL
from compas_robots.model import Joint

from compas_fab.backends.exceptions import CollisionCheckError
from compas_fab.backends.exceptions import InverseKinematicsError
from compas_fab.backends.interfaces import InverseKinematics
from compas_fab.backends.pybullet.conversions import pose_from_frame
from compas_fab.backends.pybullet.exceptions import PlanningGroupNotSupported
from compas_fab.robots import FrameTarget
from compas_fab.robots import PointAxisTarget
from compas_fab.utilities import LazyLoader

pybullet = LazyLoader("pybullet", globals(), "pybullet")


__all__ = [
    "PyBulletInverseKinematics",
]


class PyBulletInverseKinematics(InverseKinematics):
    """Mix-in functions to calculate the robot's inverse kinematics for a given target."""

    DEFAULT_TARGET_TOLERANCE_POSITION = 0.001
    DEFAULT_TARGET_TOLERANCE_ORIENTATION = 0.001

    def inverse_kinematics(self, target, robot_cell_state, group=None, options=None):
        # type: (FrameTarget, Optional[RobotCellState], Optional[str], Optional[Dict]) -> Configuration
        """Calculate the robot's inverse kinematic for a given target.

        The actual implementation can be found in the :meth:`iter_inverse_kinematics` method.
        Calling `inverse_kinematics()` will return the first solution found by the iterator,
        subsequent calls will return the next solution from the iterator. Once
        all solutions have been exhausted, the iterator will be re-initialized.

        Pybullet's inverse kinematics solver accepts FrameTarget and PointAxisTarget as input.
        The planner is a gradient descent solver, the initial position of the robot
        (supplied in the robot_cell_state) affects the first search attempt.
        Subsequent attempts will start from a random configuration, so the results may vary.

        For target-specific implementation details, see
        :meth:`_iter_inverse_kinematics_frame_target` for
        :class:`compas_fab.robots.FrameTarget` and
        :meth:`_iter_inverse_kinematics_point_axis_target` for
        :class:`compas_fab.robots.PointAxisTarget`.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which inverse kinematics is being calculated.
        target : :class:`compas_fab.robots.FrameTarget` or :class:`compas_fab.robots.PointAxisTarget`
            The target to calculate the inverse kinematics for.
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`
            The starting state to calculate the inverse kinematics for.
            The robot's configuration in the scene is taken as the starting configuration.
        group : str, optional
            The planning group used for calculation.
            Defaults to the robot's main planning group.
        options : dict, optional
            Dictionary containing kwargs for arguments specific to
            the underlying function being called.
            See the target-specific function's documentation for details.

        Raises
        ------
        :class: `compas_fab.backends.exceptions.InverseKinematicsError`
            If no configuration can be found.

        :class:`compas_fab.backends.TargetModeMismatchError`
            If the selected TargetMode is not possible with the provided robot cell state.

        Returns
        -------
        :obj:`compas_robots.Configuration`
            One of the possible IK configurations that reaches the target.

        """
        # Set default group name
        planner = self  # type: PyBulletPlanner
        client = planner.client  # type: PyBulletClient
        group = group or client.robot_cell.main_group_name

        # The caching mechanism is implemented in the iter_inverse_kinematics method
        # located in InverseKinematics class. This method is just a wrapper around it
        # so that Intellisense and Docs can point here.
        configuration = super(PyBulletInverseKinematics, self).inverse_kinematics(
            target, robot_cell_state, group, options
        )

        # After the caching, it calls the iter_inverse_kinematics method below.

        return configuration

    def iter_inverse_kinematics(self, target, robot_cell_state, group=None, options=None):
        # type: (Target, Optional[RobotCellState], Optional[str], Optional[Dict]) -> Generator[Configuration | None]
        """Calculate the robot's inverse kinematic for a given target.

        Parameters
        ----------
        target: :class:`compas.geometry.FrameTarget`
            The Frame Target to calculate the inverse for.
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`
            The starting state to calculate the inverse kinematics for.
        group: str, optional
            The planning group used for determining the end effector and labeling
            the ``start_configuration``. Defaults to the robot's main planning group.
        options: dict, optional
            Dictionary containing the following key-value pairs:

        Raises
        ------
        :class: `compas_fab.backends.exceptions.InverseKinematicsError`
            If no configuration can be found.

        :class:`compas_fab.backends.TargetModeMismatchError`
            If the selected TargetMode is not possible with the provided robot cell state.


        Yields
        ------
        :obj:`compas_robots.Configuration`
            One of the possible IK configurations that reaches the target.

        """

        planner = self  # type: PyBulletPlanner
        robot_cell = planner.client.robot_cell  # type: RobotCell
        group = group or robot_cell.main_group_name

        # Calling the super class method, which contains input sanity checks and scale normalization
        # Those are common to all planners and should be called first.
        super(PyBulletInverseKinematics, self).iter_inverse_kinematics(target, robot_cell_state, group, options)

        # ===================================================================================
        # Different target types have different implementations
        # ===================================================================================

        # Keep track of user input for checking against the final result
        initial_start_configuration = deepcopy(robot_cell_state.robot_configuration)

        if isinstance(target, FrameTarget):
            ik_generator = self._iter_inverse_kinematics_frame_target(target, robot_cell_state, group, options)
        elif isinstance(target, PointAxisTarget):
            ik_generator = self._iter_inverse_kinematics_point_axis_target(target, robot_cell_state, group, options)
        else:
            raise NotImplementedError("{} is not supported by PyBulletInverseKinematics".format(type(target)))

        # ===================================================================================
        # Check output before yielding
        # ===================================================================================

        # Check the result of the generator to detect for planning groups that are not supported by PyBullet.
        # In those cases, some joints outside of the group will be changed inadvertently.

        for configuration in ik_generator:
            # Insert any checks needed here.
            self._check_configuration_match_group(initial_start_configuration, configuration, group)
            yield configuration

    def _iter_inverse_kinematics_frame_target(self, target, robot_cell_state, group, options=None):
        # type: (FrameTarget, RobotCellState, str, Optional[Dict]) -> Generator[Configuration | None]
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
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`
            The starting state to calculate the inverse kinematics for.
        group: str
            The planning group used for determining the end effector and labeling
            the ``start_configuration``. Defaults to the robot's main planning group.
        options: dict, optional
            Dictionary containing the following key-value pairs:

            - ``"semi-constrained"``: (:obj:`bool`, optional) When ``True``, only the
              position of the target is considered. The orientation of frame will not be considered
              in the calculation.  Defaults to ``False``.
            - ``"max_descend_iterations"``:  (:obj:`float`, optional) Defines the maximum
              number of iterations to use during the gradient descend.
              If this number of iterations are reached without convergence, the solver will consider
              the initial guess as a bad starting point and will retry with a new random configuration.
              Defaults to ``20``.
              If the target tolerance is increased, this value should be increased as well.
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
            - ``"verbose"``: (:obj:`bool`, optional)
                Whether or not to print verbose output. Defaults to ``False``.

        Yields
        ------
        :obj:`compas_robots.Configuration`
            One of the possible IK configurations that reaches the target.

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

        :class:`compas_fab.backends.TargetModeMismatchError`
            If the selected TargetMode is not possible with the provided robot cell state.

        """
        options = options or {}

        # Housekeeping for intellisense
        planner = self  # type: PyBulletPlanner
        client = planner.client  # type: PyBulletClient
        robot_cell = client.robot_cell  # type: RobotCell

        # NOTE: group is not optional in this inner function.
        if group not in robot_cell.robot_semantics.groups:
            raise ValueError("Planning group '{}' not found in the robot's semantics.".format(group))

        # Default options
        options["max_descend_iterations"] = options.get("max_descend_iterations", 20)
        options["max_results"] = options.get("max_results", 100)

        options["check_collision"] = options.get("check_collision", True)
        options["return_full_configuration"] = options.get("return_full_configuration", False)
        options["solution_uniqueness_threshold_prismatic"] = options.get(
            "solution_uniqueness_threshold_prismatic", 3e-4
        )
        options["solution_uniqueness_threshold_revolute"] = options.get("solution_uniqueness_threshold_revolute", 1e-3)
        options["verbose"] = options.get("verbose", False)

        # Setting the entire robot cell state, including the robot configuration
        planner.set_robot_cell_state(robot_cell_state)

        # TODO: Implement a fail fast mechanism to check if the attached tool and objects are in collision

        # Transform the Target.target_frame to Planner Coordinate Frame depending on target.target_mode
        target_pcf = client.robot_cell.target_frames_to_pcf(
            robot_cell_state, target.target_frame, target.target_mode, group
        )

        # ===================================================================================
        # Formatting input for PyBullet
        # ===================================================================================

        body_id = client.robot_puid
        # Note: The target link is the last link in semantics.groups[group]["links"][-1]
        link_id = client.robot_link_puids[robot_cell.get_end_effector_link_name(group)]
        point, orientation = pose_from_frame(target_pcf)

        # Get list of joint_name and puids in order
        joint_names_and_puids = client._get_pose_joint_names_and_puids()
        joint_names_sorted = [joint_name for joint_name, _ in joint_names_and_puids]
        joint_ids_sorted = [joint_puid for _, joint_puid in joint_names_and_puids]

        # Prepare `rest_poses` input
        # Rest pose is PyBullet's way of defining the initial guess for the IK solver
        # The order of the values needs to match with pybullet's joint id order
        # Start configuration needs to be a full_configuration, not negotiable.
        start_configuration = robot_cell_state.robot_configuration
        all_joint_names = robot_cell.robot_model.get_configurable_joint_names()
        assert set(all_joint_names) == set(start_configuration.keys()), "Robot configuration is missing some joints"
        rest_poses = client._build_pose_for_pybullet(start_configuration)

        # Prepare `lower_limits`` and `upper_limits` input
        # Get joint limits in the same order as the joint_ids
        lower_limits = []
        upper_limits = []
        for joint_name in joint_names_sorted:
            joint = robot_cell.robot_model.get_joint_by_name(joint_name)
            lower_limits.append(joint.limit.lower if joint.type != Joint.CONTINUOUS else 0)
            upper_limits.append(joint.limit.upper if joint.type != Joint.CONTINUOUS else 2 * math.pi)

        # Prepare `jointRanges` input
        # I don't know what jointRanges needs to be.  Erwin Coumans knows, but he isn't telling.
        # https://stackoverflow.com/questions/49674179/understanding-inverse-kinematics-pybullet
        # https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/preview?pru=AAABc7276PI*zazLer2rlZ8tAUI8lF98Kw#heading=h.9i02ojf4k3ve
        joint_ranges = [u - l for u, l in zip(upper_limits, lower_limits)]

        # Prepare Parameters for calling pybullet.calculateInverseKinematics
        ik_options = dict(
            bodyUniqueId=body_id,
            endEffectorLinkIndex=link_id,
            targetPosition=point,
            targetOrientation=orientation,
            physicsClientId=client.client_id,
            lowerLimits=lower_limits,
            upperLimits=upper_limits,
            jointRanges=joint_ranges,
            restPoses=rest_poses,
        )

        # Options for semi-constrained mode, skipping the targetOrientation
        if options.get("semi-constrained"):
            ik_options.pop("targetOrientation")

        def set_random_config():
            # Function for setting random joint values for randomized search
            config = robot_cell.random_configuration(group)
            client._set_robot_configuration(config)

        # The uniqueness checker keep track of past results
        # Note that it requires the following values in options:
        # 'solution_uniqueness_threshold_prismatic' and 'solution_uniqueness_threshold_revolute'
        uniqueness_checker = UniqueResultChecker()

        # Loop to get multiple results with random restarts
        for _ in range(options.get("max_results")):

            # Calling the IK function using the helper function that repeatedly
            # calls the pybullet IK solver until convergence.

            joint_positions = self._accurate_inverse_kinematics(
                joint_ids_sorted=joint_ids_sorted,
                tolerance_position=target.tolerance_position,
                tolerance_orientation=target.tolerance_orientation,
                max_iter=options.get("max_descend_iterations"),
                verbose=options["verbose"],
                **ik_options
            )

            if not joint_positions:
                # If the solution is not close enough, we retry with a new randomized joint values
                set_random_config()
                continue

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
                    if options["verbose"]:
                        print("Collision detected. Skipping this solution.")
                        print(e)
                    # If max_results is 1, he user probably wants to know that the problem is caused by collision
                    # and not because there is no IK solution. So we re-raise the Collision Error
                    if options.get("max_results") == 1:
                        raise e
                    # If there is more attempts, we skip this solution and try again with a new randomized joint values
                    set_random_config()
                    continue
            # Construct the configuration
            return_full_configuration = options.get("return_full_configuration")
            configuration = self._build_configuration(
                joint_positions, joint_names_sorted, group, return_full_configuration, start_configuration
            )

            # Unique solution checking
            if not uniqueness_checker.check(configuration, options):
                # If the solution is not unique, we retry with a new randomized joint values
                set_random_config()
                continue

            # If we got this far, we have a valid solution to yield
            yield configuration

            # In order to generate multiple IK results,
            # we start the next loop iteration with randomized joint values
            set_random_config()

        # If no solution was found after max_results, raise an error
        if len(uniqueness_checker.results) == 0:
            raise InverseKinematicsError(
                "No solution found after {} attempts (max_results).".format(options.get("max_results")),
                target_pcf=target_pcf,
            )

    def _iter_inverse_kinematics_point_axis_target(self, target, robot_cell_state, group, options=None):
        # type: (PointAxisTarget, RobotCellState, str, Optional[Dict]) -> Generator[Configuration | None]
        """Calculate the robot's inverse kinematic for a given PointAxisTarget.

        class:`~compas_fab.robots.PointAxisTarget` specify a target point and an axis in 3D space.
        Depending on setting of `target.target_mode`, the selected reference frame (e.g. TCF)
        on the robot will be moved such that the `frame.point` is at the target point and the
        `frame.zaxis` is aligned with the target axis.
        The reference frame is allowed to rotate around the axis.
        For a typical 6-DOF robot, a PointAxisTarget will have 1 extra degree of freedom
        compared to a FrameTarget.

        Similar to the :meth:`_iter_inverse_kinematics_frame_target` method, this function
        will make multiple attempts to find a solution that is reachable and collision free.

        Given the extra degree of freedom, the solver will discretize the rotation around
        the axis into multiple steps. The number of steps is determined by the ``num_rotation_steps``.
        The initial rotation (theta = 0) is determined by a minimally rotated frame computed
        from the robot's starting configuration (``robot_cell_state``).
        From this initial frame, the solver will test other frames
        by rotating the frame around the axis in equal steps until a solution is found.
        The amount of rotation is incremented left and right alternatively from zero (the initial frame).
        For example, the theta values (in degrees) for a 36-step rotation will be [0, 10, -10, 20, -20, ...].

        When all rotational steps are exhausted. The solver will then restart the entire search
        from a new random configuration and repeat the process until the maximum number of attempts
        (``max_random_restart``) is reached.
        This also means that the generator will behave like a random search after the first round
        of rotation steps if  ``max_random_restart`` is greater than 1.

        This function internally calls the :meth:`_iter_inverse_kinematics_frame_target` method,
        which is responsible for the actual IK calculation after the rotation is fixed
        by this function. Some of the options passed to this function will be passed on to the
        :meth:`_iter_inverse_kinematics_frame_target` method, see below for details.

        Parameters
        ----------
        target: :class:`compas.geometry.PointAxisTarget`
            The PointAxis Target to calculate the inverse for.
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`
            The starting state to calculate the inverse kinematics for.
        group: str
            The planning group used for calculation.
        options: dict, optional
            Dictionary containing the following key-value pairs, that are unique to this function:

            - ``"num_rotation_steps"``: (:obj:`int`, optional)
              Number of steps to rotate the frame around the axis.
              Defaults to ``20``.
            - ``"max_random_restart"``: (:obj:`int`, optional)
              Maximum number of random restarts with random configuration to find a solution.
              Defaults to ``5``.
            - ``"max_results"``: (:obj:`int`)
              Maximum number of results to return before stopping the search.
              Defaults to ``100``.
            - ``"max_results_per_restart"``: (:obj:`int`, optional)
              Maximum number of results to return before restarting the search from a new random configuration.
              This optional control for search behavior is useful when the search space is large.
              Reducing this value will make the search favor random restarts over exhaustive search.
              The value should be between 1 and num_rotation_steps.
              Defaults to the value of ``num_rotation_steps``.

            The following key-value pairs have the same meaning as in the :meth:`_iter_inverse_kinematics_frame_target` method:

            - ``"max_descend_iterations"``:  (:obj:`float`, optional)
            - ``"solution_uniqueness_threshold_prismatic"``: (:obj:`float`, optional)
            - ``"solution_uniqueness_threshold_revolute"``: (:obj:`float`, optional)
            - ``"check_collision"``: (:obj:`bool`, optional)
            - ``"return_full_configuration"``: (:obj:`bool`, optional)
            - ``"verbose"``: (:obj:`bool`, optional)

        Notes
        -----

        The search space for PointAxisTarget is much larger than FrameTarget, so the number of attempts
        and the number of rotation steps should be kept low to avoid long search times.
        Be aware that if the target is not reachable or caused collision, the search will iterate through
        all the steps and restarts before giving up.

        Yields
        ------
        :obj:`compas_robots.Configuration`
            One of the possible IK configurations that reaches the target.

        Raises
        ------
        :class:`compas_fab.backends.InverseKinematicsError`
            Indicates that no IK solution could be found by the kinematic solver
            after the maximum number of attempts (``max_results``). This can be caused by
            reachability or collision or both.

        """
        options = options or {}

        # Housekeeping for intellisense
        planner = self  # type: PyBulletPlanner
        client = planner.client  # type: PyBulletClient
        robot_cell = client.robot_cell  # type: RobotCell

        # NOTE: group is not optional in this inner function.
        if group not in robot_cell.robot_semantics.groups:
            raise ValueError("Planning group '{}' not found in the robot's semantics.".format(group))

        # Default options (specific to PointAxisTarget)
        options["num_rotation_steps"] = options.get("num_rotation_steps", 20)
        options["max_random_restart"] = options.get("max_random_restart", 5)
        options["max_results"] = options.get("max_results", 100)
        # NOTE: The option 'max_results' have a different meaning to the one
        #       in the _iter_inverse_kinematics_frame_target, here we keep a count of the
        #       number of returned result and use it to determine when to stop the search.
        #       In the _iter_inverse_kinematics_frame_target, it is used to determine the
        #       maximum number of attempts to find a solution.

        #       In this function, the maximum number of attempts is determined by the
        #       'num_rotation_steps' and 'max_random_restart' options.
        options["max_results_per_restart"] = options.get("max_results_per_restart", options["num_rotation_steps"])

        # Default options (for the FrameTarget function)
        # Later there is a frame_ik_options dict that will be passed to the pybullet IK solver
        # That one will set the max_results to 1 for the FrameTarget function
        options["max_descend_iterations"] = options.get("max_descend_iterations", 20)

        options["solution_uniqueness_threshold_prismatic"] = options.get(
            "solution_uniqueness_threshold_prismatic", 3e-4
        )
        options["solution_uniqueness_threshold_revolute"] = options.get("solution_uniqueness_threshold_revolute", 1e-3)
        options["check_collision"] = options.get("check_collision", True)
        options["return_full_configuration"] = options.get("return_full_configuration", False)
        options["verbose"] = options.get("verbose", False)

        # Setting the entire robot cell state, including the robot configuration
        planner.set_robot_cell_state(robot_cell_state)

        def compute_initial_frame(configuration):
            """Compute the first target frame guided by FK from initial robot configuration.
            This function reduces the searching time by prioritizing the first search frame
            to be close to the initial robot configuration.
            """

            # Create intermediate state and call FK
            robot_cell_state.robot_configuration = configuration

            # The reference frame for guiding the rotation must match (PCF/TCF/OCF)
            # with the reference frame specified by the target_mode.
            target_guide_frame = planner.forward_kinematics(robot_cell_state, target.target_mode, group)

            # We need to check if the FK X axis is parallel to the target.axis
            is_parallel = is_parallel_vector_vector(target_guide_frame.xaxis, target.target_z_axis)

            # If parallel (which should be rare), we use the y-axis of the FK frame as the guide vector
            # This vector will be the Y direction for the first rotated frame
            if is_parallel:
                target_x_axis = target.target_z_axis.cross(target_guide_frame.yaxis).inverted()
                target_y_axis = target.target_z_axis.cross(target_x_axis)
            # If not, we use this guide vector as X direction for the first rotated frame
            else:
                target_y_axis = target.target_z_axis.cross(target_guide_frame.xaxis)
                target_x_axis = target.target_z_axis.cross(target_y_axis).inverted()

            # Construct and return the initial frame
            initial_frame = Frame(target.target_point, target_x_axis, target_y_axis)
            return initial_frame

        # Extract initial start configuration from the robot cell state
        # Subsequent configurations will be randomized
        start_configuration = robot_cell_state.robot_configuration
        assert start_configuration, "Robot configuration is missing"

        def set_random_config():
            # Function for setting random joint values for randomized search
            start_configuration = robot_cell.random_configuration(group)
            client._set_robot_configuration(start_configuration)

        # Options dict for passing to the FrameTarget function
        frame_ik_options = options.copy()
        # Max result must be 1 for the underlying IK function to not perform random-restart
        frame_ik_options["max_results"] = 1
        # Semi-constrained mode does not make any sense in our use case, so we disable it
        # in case the user has set it to True
        frame_ik_options["semi-constrained"] = False

        # The uniqueness checker keep track of past results
        uniqueness_checker = UniqueResultChecker()

        # Iterate through the number of random restarts
        for _ in range(options.get("max_random_restart")):

            # If the number of results is more than the max_results, we stop the search
            if len(uniqueness_checker.results) >= options["max_results"]:
                break

            # The Initial frame is the 0th rotation frame
            # Note that the initial frame is generated after some random configuration is set,
            # there will be some randomness in the initial frame too.
            initial_frame = compute_initial_frame(start_configuration)

            # The rotation is incremented left and right alternatively from zero
            # For example, the theta values (in degrees) for a 36-step rotation will be [0, 10, -10, 20, -20, ...]
            rotation_step_size = 2 * math.pi / options["num_rotation_steps"]
            for step in range(options["num_rotation_steps"]):
                # Advanced option to limit the number of results in a single restart
                results_in_this_restart = 0
                if results_in_this_restart >= options["max_results_per_restart"]:
                    break

                theta = rotation_step_size * ((step + 1) // 2) * (-1) ** (step % 2)
                rotated_frame = initial_frame.rotated(theta, initial_frame.zaxis, initial_frame.point)
                if options["verbose"]:
                    print("Rotated Frame: {}, theta: {}".format(rotated_frame, theta))
                frame_target = FrameTarget(
                    rotated_frame, target.target_mode, target.tolerance_position, target.tolerance_orientation
                )

                # Call underlying FrameTarget function to get IK solutions
                ik_frame_target = self._iter_inverse_kinematics_frame_target(
                    frame_target, robot_cell_state, group, frame_ik_options
                )
                # We will only get one result from the FrameTarget function, if that result is None, we skip to the next rotation
                try:
                    configuration = next(ik_frame_target)
                    # Uniqueness checking
                    if not uniqueness_checker.check(configuration, options):
                        continue

                    # If we got this far, we have a valid solution to yield
                    results_in_this_restart += 1
                    yield configuration
                except InverseKinematicsError:
                    continue
                except CollisionCheckError:
                    continue

            # If no solution was found after the rotation steps, we restart the search
            # by setting a new random configuration
            set_random_config()

        # If no solution is found after everything is exhausted, raise an error
        if len(uniqueness_checker.results) == 0:
            raise InverseKinematicsError(
                "No solution found after {} attempts (max_random_restart).".format(options.get("max_random_restart"))
            )

    def _accurate_inverse_kinematics(
        self, joint_ids_sorted, max_iter, tolerance_position=None, tolerance_orientation=None, verbose=False, **kwargs
    ):
        """Iterative inverse kinematics solver with a threshold for the distance to the target.

        This functions helps to get a more accurate solution by iterating over the IK solver
        until the distance to the target is below a certain threshold.
        This overcomes the limitations of the default pybullet solver, which does not guarantee any accuracy.

        Returns
        -------
        tuple of list of float, bool
            A tuple containing the joint positions and a boolean indicating if the solution is close enough to the target.
        """
        # NOTE: In principle, this accurate iter IK should work out of the
        # pybullet box, but the results seem to be way off target. For now,
        # I'm leaving the legacy iterative accurate ik in python as per
        # older examples of pybullet, until we figure out why the builtin
        # one is not cooperating.

        # Based on these examples
        # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_kinematics_husky_kuka.py#L81
        body_id = kwargs["bodyUniqueId"]
        link_id = kwargs["endEffectorLinkIndex"]

        # Target position (Point) and orientation (convert to Quaternion)
        target_position = kwargs["targetPosition"]
        target_orientation_xyzw = kwargs.get("targetOrientation", None)
        if target_orientation_xyzw:
            _x, _y, _z, _w = target_orientation_xyzw
            target_quaternion = Quaternion(_w, _x, _y, _z)
            target_orientation_frame = Frame.from_quaternion(target_quaternion)
        else:
            target_quaternion = None

        tolerance_position = tolerance_position or self.DEFAULT_TARGET_TOLERANCE_POSITION
        tolerance_orientation = tolerance_orientation or self.DEFAULT_TARGET_TOLERANCE_ORIENTATION
        # Note: the following two options that was present in the PyBullet IK Example does not seem to do anything
        # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_kinematics.py
        # kwargs["maxNumIterations"] = max_iter
        # kwargs["residualThreshold"] = threshold

        for i in range(max_iter):
            joint_poses = pybullet.calculateInverseKinematics(**kwargs)
            for i in range(len(joint_ids_sorted)):
                pybullet.resetJointState(body_id, joint_ids_sorted[i], joint_poses[i])
            # Retrieve the last link state that contains the last link's position (index 4) and orientation (index 5)
            link_state = pybullet.getLinkState(body_id, link_id)

            # Check tolerance_position
            new_position = link_state[4]
            diff = [
                target_position[0] - new_position[0],
                target_position[1] - new_position[1],
                target_position[2] - new_position[2],
            ]
            # The distance is squared to avoid a sqrt operation
            # Therefor, the threshold is squared as well
            distance_squared = diff[0] ** 2 + diff[1] ** 2 + diff[2] ** 2
            tolerance_position_ok = distance_squared < tolerance_position**2
            # print("Iter: %d, Distance: %s" % (i, distance_squared))

            # Check tolerance_orientation
            if target_quaternion:
                _x, _y, _z, _w = link_state[5]
                new_quaternion = Quaternion(_w, _x, _y, _z)
                new_frame = Frame.from_quaternion(new_quaternion)
                delta_frame = target_orientation_frame.to_local_coordinates(new_frame)
                _, angle = axis_angle_from_quaternion(Quaternion.from_frame(delta_frame))

                tolerance_orientation_ok = angle < tolerance_orientation
                # print("Iter: %d, Angle: %s" % (i, angle_squared))
            else:
                tolerance_orientation_ok = True

            if tolerance_position_ok and tolerance_orientation_ok:
                if verbose:
                    print("Iterative IK took %d iterations" % i)
                return joint_poses
            kwargs["restPoses"] = joint_poses
            i += 1

        return None

    def _check_configuration_match_group(self, start_configuration, configuration, group):
        # type: (Configuration, Configuration, str) -> None
        """Check if the configuration changed only the joints in the group.

        We assume that if a joint value is changed inadvertently, it is because
        the planning group is not supported by PyBullet.

        Parameters
        ----------
        start_configuration : :class:`compas_robots.Configuration`
            The initial configuration.
        configuration : :class:`compas_robots.Configuration`
            The configuration to check.
        group : str
            The planning group to check. Not optional, must be specified.

        Raises
        ------
        KeyError
            If the configuration has joints that are not in the start configuration.
        ValueError
            If the configuration has changed joints that are not in the group.
        """
        robot_cell = self.client.robot_cell  # type: RobotCell

        configurable_joints = robot_cell.get_configurable_joint_names(group)
        for joint_name, joint_value in configuration.items():
            if joint_name not in start_configuration:
                raise KeyError(
                    "Configuration has joint '{}' that is not in the start configuration.".format(joint_name)
                )
            if joint_name in configurable_joints:
                continue
            if not TOL.is_close(joint_value, start_configuration[joint_name]):
                joint_names = robot_cell.robot_semantics.groups[group]["joints"]
                link_names = robot_cell.robot_semantics.groups[group]["links"]
                print("Configuration changed joint '{}' that is not in the group.".format(joint_name))
                raise PlanningGroupNotSupported(group, joint_names, link_names)


class UniqueResultChecker(object):
    """A class to check if a configuration is unique.

    This class will keep track of past solutions and compare new solutions with them.
    The values are compared with a threshold that is different for each joint type.


    Attributes
    ----------
    sorted_uniqueness_thresholds : list of float
        The thresholds for each joint to consider the solution unique.
        One value for each joint in the same order as the joint_positions.
    results : list of :class:`compas_robots.Configuration`
        The past accepted configurations to compare with.
        This list will be appended automatically when calling the check method.
        Only those that passes the check will be added to this list.
    """

    def __init__(self):
        self.sorted_uniqueness_thresholds = None  # type: List[float]
        self.results = []  # type: List[Configuration]

    def check(self, configuration, options):
        # type: (Configuration, Dict) -> bool
        """Check if the solution is unique by comparing with past solutions.

        Accepted solutions will be added to the `self.results` list.
        Therefore, this method should be called only once for each solution.
        Second call with the same solution will certainly return False.

        Parameters
        ----------
        configuration : :class:`compas_robots.Configuration`
            The configuration to be checked.
        options : dict
            The options for the uniqueness thresholds.
            It should contain the following keys:

            - ``"solution_uniqueness_threshold_prismatic"``: :obj:`float`
              The minimum distance between two solutions in the prismatic joint space to consider them unique.
              Units are in meters.
            - ``"solution_uniqueness_threshold_revolute"``: :obj:`float`
              The minimum distance between two solutions in the revolute joint space to consider them unique.
              Units are in radians.

        """
        # First check is a fast pass to avoid unnecessary calculations
        if not self.results:
            self.results.append(configuration)
            return True

        # From second check onwards, we compare with past solutions
        # First the uniqueness thresholds are computed
        if self.sorted_uniqueness_thresholds is None:
            self.compute_sorted_uniqueness_thresholds(configuration, options)

        # Compare the new solution with past solutions
        for past_configuration in self.results:
            # Only if all joints are same, we consider the solution as not unique
            if all(
                TOL.is_close(joint_position, past_joint_position, atol=threshold)
                for joint_position, past_joint_position, threshold in zip(
                    configuration.joint_values, past_configuration.joint_values, self.sorted_uniqueness_thresholds
                )
            ):
                return False
        self.results.append(configuration)
        return True

    def compute_sorted_uniqueness_thresholds(self, configuration, options):
        # type: (Configuration, Dict) -> UniqueResultChecker
        """Create a UniqueResultChecker from a configuration.

        Parameters
        ----------
        config : :class:`compas_robots.Configuration`
            A configuration that contains a ordered list of joints and their types.
        """
        joint_types = configuration.joint_types
        sorted_uniqueness_thresholds = [
            (
                options["solution_uniqueness_threshold_prismatic"]
                if joint_type in [Joint.PRISMATIC, Joint.PLANAR]
                else options["solution_uniqueness_threshold_revolute"]
            )
            for joint_type in joint_types
        ]
        self.sorted_uniqueness_thresholds = sorted_uniqueness_thresholds
