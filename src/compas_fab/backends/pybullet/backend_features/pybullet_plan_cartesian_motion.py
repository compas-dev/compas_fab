from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from copy import deepcopy
from math import ceil
from math import pi

from compas import IPY
from compas.geometry import Quaternion
from compas.geometry import axis_angle_from_quaternion
from compas.geometry import cross_vectors
from compas.geometry import is_parallel_vector_vector
from compas_robots.model import Joint

from compas_fab.backends import CollisionCheckError
from compas_fab.backends import InverseKinematicsError
from compas_fab.backends import MPInterpolationInCollisionError
from compas_fab.backends import MPMaxJumpError
from compas_fab.backends import MPNoIKSolutionError
from compas_fab.backends import MPNoPlanFoundError
from compas_fab.backends import MPStartStateInCollisionError
from compas_fab.backends import MPTargetInCollisionError
from compas_fab.robots import FrameTarget
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import JointTrajectory
from compas_fab.robots import JointTrajectoryPoint
from compas_fab.robots import PointAxisTarget
from compas_fab.robots import PointAxisWaypoints

from .helpers import check_max_jump

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:  # pragma: no cover
        from typing import Dict  # noqa: F401
        from typing import List  # noqa: F401
        from typing import Optional  # noqa: F401
        from typing import Tuple  # noqa: F401

        from compas.geometry import Frame  # noqa: F401
        from compas.geometry import Point  # noqa: F401
        from compas.geometry import Vector  # noqa: F401
        from compas_robots import Configuration  # noqa: F401

        from compas_fab.backends import PyBulletClient  # noqa: F401
        from compas_fab.backends import PyBulletPlanner  # noqa: F401
        from compas_fab.robots import RobotCell  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import Waypoints  # noqa: F401


__all__ = [
    "PyBulletPlanCartesianMotion",
]
from compas_fab.backends.interfaces import PlanCartesianMotion


class PyBulletPlanCartesianMotion(PlanCartesianMotion):
    """Callable to calculate a cartesian motion path (linear in tool space)."""

    def plan_cartesian_motion(self, waypoints, start_state, group=None, options=None):
        # type: (Waypoints, RobotCellState, Optional[str], Optional[Dict]) -> JointTrajectory
        """Calculates a cartesian motion path (linear in tool space) for Waypoints.

        Supports FrameWaypoints and PointAxisWaypoints.

        For more information such as target specific behaviors, see
        :meth:`~compas_fab.backends.PyBulletClient.plan_cartesian_motion_point_axis_waypoints` for
        :class:`~compas_fab.robots.PointAxisWaypoints` and
        :meth:`~compas_fab.backends.PyBulletClient.plan_cartesian_motion_frame_waypoints` for
        :class:`~compas_fab.robots.FrameWaypoints`.

        Note that the starting state of the robot cell must match with the objects in the robot cell
        previously set using :meth:`compas_fab.backends.PyBulletClient.set_robot_cell`.
        When a tool is attached to the planning group (specified in the start_state), it is possible
        to use `Waypoints.target_mode = TargetMode.TOOL`.
        When a workpiece is attached to the said tool, it is possible to use
        `Waypoints.target_mode = TargetMode.WORKPIECE`.

        The robot's full configuration, i.e. values for all configurable joints of the entire robot,
        must be provided in the ``start_state.robot_configuration`` parameter.

        The ``waypoints`` parameter is used to defined single or multiple segments of path,
        it is not necessary to include the starting pose in the waypoints,
        doing so may result in a redundant trajectory point.

        Each path segment is interpolated linearly in Cartesian space, where position is interpolated linearly in Cartesian space
        and orientation is interpolated spherically using quaternion interpolation.

        If the planning is successful, the function will return a :class:`compas_fab.robots.JointTrajectory` object,
        which contains the start_configuration (same as input) and a list of :class:`compas_fab.robots.JointTrajectoryPoint`.

        - Note that the first point in the trajectory will not repeat the start_configuration.
        - Note that there may be one or more trajectory point for each path segment due to interpolation.
        - Note that the returned trajectory does not separate the trajectory points into segments based on the waypoints.
          If the user wants to separate the path into segments, they should consider calling this function multiple times.

        The ``check_collision`` parameter can be used to enable or disable collision checking during the planning process.
        If enabled, the planner will check for collision at each trajectory point
        similar to the :meth:`compas_fab.backends.PyBulletClient.check_collision` method.


        Parameters
        ----------
        waypoints : :class:`compas_fab.robots.Waypoints`
            The waypoints for the robot to follow.
        start_state : :class:`compas_fab.robots.RobotCellState`
            The starting state of the robot cell at the beginning of the motion.
            The attribute `robot_configuration`, must be provided.
        group: str, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        options: dict, optional
            Dictionary containing additional options, see specific planner for details.

            - ``"check_collision"``: (:obj:`str`, optional) When ``True``,
              :meth:`compas_fab.backends.PyBulletCheckCollision.check_collision` will be called.
              Defaults to ``True``. Setting this to False may help during interactive debugging.

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
            The calculated trajectory.

        Raises
        ------
        :class:`compas_fab.backends.CollisionCheckError`
            If ``check_collision`` is enabled and one of the configurations in the path is in collision.

        :class:`compas_fab.backends.InverseKinematicsError`
            Indicates that one point along the path has no IK solution.

        :class:`compas_fab.backends.TargetModeMismatchError`
            If the selected TargetMode is not possible with the provided robot cell state.

        Notes
        -----
        This planning function is synchronous, meaning that it will block the main thread until the planning is complete.
        If the user wants to run the planning in the background and be able to cancel it, they should consider wrapping
        this function in a separate thread or process.

        """

        # NOTE: The conversion of Waypoints Targets from TCF to T0CF must happen inside the
        # corresponding planner method. This is because for PointAxisWaypoints, the planner
        # must be able to spin around the TCF Z axis freely.
        # This is not possible anymore if the targets are converted to T0CF representation here.

        # ===================================================================================
        # The following lines should be typical in all planners' plan_cartesian_motion method
        # ===================================================================================

        planner = self  # type: PyBulletPlanner
        client = planner.client  # type: PyBulletClient
        robot_cell = client.robot_cell  # type: RobotCell
        group = group or robot_cell.main_group_name

        # Unit conversion from user scale to meter scale can be done here because they are shared by all planners.
        waypoints = waypoints.normalized_to_meters()

        # Check if the robot cell state supports the target mode
        planner = self  # type: PyBulletPlanner
        start_state.assert_target_mode_match(waypoints.target_mode, group)

        # Check start_state is formatted correctly
        if not start_state.robot_configuration:
            raise ValueError("start_state.robot_configuration must be provided.")
        if not start_state.robot_configuration.joint_names:
            raise ValueError("start_state.robot_configuration.joint_names must be provided.")
        client.robot_cell.assert_cell_state_match(start_state)

        # Get default group name if not provided
        # Do not skip this line because some functions do not default to main_group_name when group input is None.
        group = group or robot_cell.main_group_name

        # ===================================================================================
        # End of common lines
        # ===================================================================================

        if isinstance(waypoints, PointAxisWaypoints):
            return self.plan_cartesian_motion_point_axis_waypoints(waypoints, start_state, group, options)
        elif isinstance(waypoints, FrameWaypoints):
            return self.plan_cartesian_motion_frame_waypoints(waypoints, start_state, group, options)
        else:
            raise NotImplementedError("Only PointAxisWaypoints and FrameWaypoints are supported.")

    def plan_cartesian_motion_point_axis_waypoints(self, waypoints, start_state, group, options=None):
        # type: (PointAxisWaypoints, RobotCellState, Optional[str], Optional[Dict]) -> JointTrajectory
        """Calculates a cartesian motion path (linear in tool space) for Point Axis Waypoints.

        Similar to the :meth:`~plan_cartesian_motion_frame_waypoints` method, this method calculates a cartesian motion path
        (linear in tool space) for Point Axis Waypoints. The main difference is that the interpolation is
        performed with the point and axis of the target. The main application of this function is
        for 3D printing, milling, or other applications where the tool is cylindrical and must follow a specific path.
        Users should select the correct target_mode in the waypoints to ensure that the planner interpolates correctly,
        It is common to use TargetMode.TOOL for the example applications mentioned above, such that the reference frame
        is the tool coordinate frame (TCF).
        Note that the Z axis of the reference frame is aligned with the axis of the target.
        Users who wish to use other axis should consider redefining the ToolModel.frame in their tools.

        Users can choose to provide waypoints densely or sparsely.
        High density waypoints will allow the user to control precisely the path of the tool, such as
        when following a curve. This is because the planner is guaranteed to generate a JointTrajectoryPoint
        exactly at each waypoint. Sparse waypoints is suitable when the tool traces a straight line, the planner
        will create interpolated points between the waypoints automatically.

        The interpolated points and axis have regular spacing between each waypoint segment.
        The interpolation spacing is controlled by the ``max_step_distance`` and ``max_step_angle`` options,
        they control the distance between points and the angle between axis. This setting affects the smoothness
        of the trajectory.

        Unlike the Cartesian motion planning with Frame Waypoints, the planner will not perform sub-division
        of the interpolation when the joint jump is too large. It will simply reject the IK solution and try a different
        tool orientation. Therefore, the user should not set the ``max_jump`` parameters too low, as it may prevent the
        planner from finding a solution. For application that are not bounded by the joint speed, the user can set the
        ``max_jump`` parameters to a high value (e.g. 0.1 meter and 1 pi radian), simply as a means to avoid sudden joint flips.

        For applications that require a specific tool speed, the user can use the max_jump parameters as a rudimentary way
        of constraining the maximum difference between two consecutive points, hence keeping speed within the desired range.

        The planning algorithm starts searching from the starting configuration and iteratively tries to find the next IK solution
        in the forward direction. When searching for the next IK solution, the planner will use the last configuration to perform
        a gradient decent, this is to ensure that the next configuration is close to the last one. However, this also limits the
        planner to not be able to jump to another configuration (e.g. from elbow up to elbow down). Users should therefore be
        aware that the planner is incomplete and may not find certain solutions even if one exists.

        Because of the iterative search, the search space and searching time is highly sensitive to the starting configuration.
        A good starting configuration can greatly reduce the search time, while a bad starting configuration can make the search
        impossible. In general the user should provide a starting configuration that is likely to transition smoothly to all the
        waypoints.

        The planner can be configured to sample the starting configuration to improve the chance of finding a solution,
        this is controlled by the ``sample_start_configuration`` option. If enabled, the planner will treat the starting
        configuration as an freely rotatable point-axis target. If a solution is found, the sampled configuration will be
        returned in trajectory.start_configuration. In this mode, the start_state.robot_configuration is ignored.

        Notes
        -----
        Users should call :meth:`~PyBulletPlanCartesianMotion.plan_cartesian_motion` instead of this method directly.
        Doing so will ensure that necessary scaling and checks are performed.

        Parameters
        ----------
        waypoints : :class:`compas_fab.robots.PointAxisWaypoints`
            The waypoints for the robot to follow.
        start_state : :class:`compas_fab.robots.RobotCellState`
            The starting state of the robot cell at the beginning of the motion.
            The attribute `robot_configuration`, must be provided.
        group: str
            The planning group used for calculation.
        options: dict, optional
            Dictionary containing the following key-value pairs:

            - ``"max_step_distance"``: (:obj:`float`, optional)
              The max Cartesian distance between two consecutive points in the result.
              Unit is in meters, defaults to ``0.01``.
            - ``"max_step_angle"``: (:obj:`float`, optional)
              The max angular distance between two consecutive points in the result.
              Unit is in radians, defaults to ``0.1``.
            - ``"sample_start_configuration"``: (:obj:`bool`, optional)
              Whether or not to sample the start configuration. Defaults to ``False``.
            - ``"check_collision"``: (:obj:`bool`, optional)
              Whether or not to avoid collision. Defaults to ``True``.
            - ``"max_jump_prismatic"``: (:obj:`float`, optional)
              The maximum allowed distance of prismatic joint positions
              between consecutive trajectory points.
              Unit is in meters, defaults to ``0.1``.
              Setting this to ``0`` will disable this check.
            - ``"max_jump_revolute"``: (:obj:`float`, optional)
              The maximum allowed distance of revolute and continuous joint positions
              between consecutive trajectory points.
              Unit is in radians, defaults to :math:`\\pi / 2` (90 degrees).
              Setting this to ``0`` will disable this check.
            - ``"check_collision"``: (:obj:`bool`, optional)
              Whether or not to avoid collision. Defaults to ``True``.
            - ``"verbose"``: (:obj:`bool`, optional)
                Whether or not to print verbose output. Defaults to ``True``.
            - ``"skip_preplanning_collision_check"``: (:obj:`bool`, optional)
                Whether or not to skip the target check before planning. Defaults to ``True``.
                The preplanning check in intended to provide a fail-fast feedback for the user.
                However, if there are many targets in the waypoints, the check maybe more
                time consuming than the benefit it provides.
                Because this planner is often used for 3D printing, milling, etc. the user is likely
                to provide dense waypoints, therefore this check is disabled by default.

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
            The calculated trajectory.

        Raises
        ------
        :class:`compas_fab.backends.MPNoIKSolutionError`
            If no IK solution could be found by the kinematic solver.
            The partially planned trajectory before the unreachable target is returned.
            The unreachable target frame is returned.

        """

        # Set default option values
        options = options or {}
        options = deepcopy(options)
        options["max_step_distance"] = options.get("max_step_distance", 0.01)  # meters
        options["max_step_angle"] = options.get("max_step_angle", 0.1)  # radians
        options["sample_start_configuration"] = options.get("sample_start_configuration", False)
        options["check_collision"] = options.get("check_collision", True)
        options["max_jump_prismatic"] = options.get("max_jump_prismatic", 0.1)
        options["max_jump_revolute"] = options.get("max_jump_revolute", pi / 2)
        options["skip_preplanning_collision_check"] = options.get("skip_preplanning_collision_check", True)
        options["verbose"] = options.get("verbose", False)

        # Housekeeping for intellisense
        planner = self  # type: PyBulletPlanner
        client = planner.client  # type: PyBulletClient
        robot_cell = client.robot_cell  # type: RobotCell

        # Setting robot cell state
        planner.set_robot_cell_state(start_state)

        # TODO: Check if the start state is in collision

        # TODO: Check if the attached tool and workpiece are in collision at every waypoint

        # Options for Inverse Kinematics
        ik_options = deepcopy(options)

        # NOTE: # We only need the joint values for the group to construct the JointTrajectoryPoint
        ik_options["max_results"] = 1
        # NOTE: The PyBullet IK function is responsible for performing collision checking for this planning function
        # The ["check_collision"] in options is passed also to the ik_options
        ik_options["return_full_configuration"] = False
        # NOTE: Turning max_random_restart to 1 because a random restart is very unlikely to be continuous with the
        # previous configuration. This will just waste search time for solutions that fail the max_jump check.
        # In addition, setting this to 1 will make the search deterministic.
        ik_options["max_random_restart"] = 1

        # Getting the joint names this way ensures that the joint order is consistent with Semantics
        joint_names = robot_cell.get_configurable_joint_names(group)
        joint_types = robot_cell.get_configurable_joint_types(group)

        # Iterate over the waypoints as segments
        intermediate_state = deepcopy(start_state)  # type: RobotCellState
        start_configuration = start_state.robot_configuration
        # TODO: We currently trust that the input configuration has a correct joint order, this should be checked

        # Recreate the first frame for the beginning of the interpolation
        fk_options = deepcopy(options)
        fk_options["link"] = robot_cell.get_end_effector_link_name(group)

        # This frame reference need to match with the target_mode of the waypoints
        start_frame = planner.forward_kinematics(start_state, waypoints.target_mode, group=group, options=fk_options)
        if options["verbose"]:
            print("Reconstructed Start Frame:", start_frame)

        starting_target = PointAxisTarget(
            start_frame.point,
            start_frame.zaxis,
            target_mode=waypoints.target_mode,
            tolerance_position=waypoints.tolerance_position,
            tolerance_orientation=waypoints.tolerance_orientation,
        )

        # Interpolation is performed for all the waypoints before planning.
        # There will be no further sub-division.
        # This makes the DFS code easier to implement.

        all_targets = [starting_target]  # type: List[PointAxisTarget]
        for i in range(len(waypoints.target_points_and_axes)):
            start_point_axis = (
                waypoints.target_points_and_axes[i - 1] if i > 0 else (start_frame.point, start_frame.zaxis)
            )
            end_point_axis = waypoints.target_points_and_axes[i]
            interpolator = PointAxisInterpolator(start_point_axis, end_point_axis, options)

            # Compute the number of steps for the interpolation, step == 1 means no interpolation.
            steps = interpolator.regular_interpolation_steps
            for j in range(steps):
                # t == 0 is skipped because it is the same as the last target
                # t == 1 represent the end target
                t = (j + 1) / steps
                interpolated_point, interpolated_axis = interpolator.get_interpolated_point_axis(t)
                target = PointAxisTarget(
                    interpolated_point,
                    interpolated_axis,
                    target_mode=waypoints.target_mode,
                    tolerance_position=waypoints.tolerance_position,
                    tolerance_orientation=waypoints.tolerance_orientation,
                )
                all_targets.append(target)

            if options["verbose"]:
                print("Interpolation: {} steps towards Waypoint {}".format(steps, i))

        def _build_return_trajectory(configurations):
            # Create the trajectory
            trajectory = JointTrajectory(joint_names=joint_names, start_configuration=start_configuration)
            for planned_configuration in configurations:
                joint_values = [planned_configuration[joint_name] for joint_name in joint_names]
                trajectory.points.append(
                    JointTrajectoryPoint(joint_values=joint_values, joint_types=joint_types, joint_names=joint_names)
                )
            return trajectory

        # Perform Depth First Search (DFS) over the targets to plan the trajectory

        # ===================================================================================
        # DFS Code without recursion
        # ===================================================================================

        # TODO: Implement sample_start_configuration
        current_step = 1  # This is the current step in the DFS
        planned_configurations = [start_configuration]  # This list holds the planned configurations for each step
        ik_generators = [zip()]  # This list holds the iterative ik generators for each step
        ik_option_indices = [0]  # This is only used for debugging
        longest_list_of_planned_configurations = (
            []
        )  # This is used to store the longest trajectory found and return it when search fails
        comprehensively_checked = []  # This stores the target indices that have been checked for IK solutions

        while True:
            # Create a generator if the current step does not have one
            if current_step >= len(ik_generators):
                intermediate_state = deepcopy(
                    start_state
                )  # Deep copy because we will have multiple generators in the stack
                intermediate_state.robot_configuration = planned_configurations[-1]
                ik_generator = planner._iter_inverse_kinematics_point_axis_target(
                    all_targets[current_step], intermediate_state, group, ik_options
                )
                ik_generators.append(ik_generator)
                ik_option_indices.append(-1)
                if options["verbose"]:
                    print("DFS Step: {} of {}. Created IK Generator".format(current_step, len(all_targets) - 1))

            # Check if the IK generator (of current step) has any more options
            try:
                ik_result = next(ik_generators[-1], None)  # type: Configuration | None
                ik_option_indices[-1] += 1
            except InverseKinematicsError:
                # If the generator raises an IK error, it means that there is not even one valid IK solution
                # If this is the case, backtrack is not going to help solve the problem and we should stop the search
                ik_result = None
                ik_option_indices[-1] += 1
                # In order to save searching time, we perform a more comprehensive check here to determine if this
                # target is completely unreachable even with random starting configurations.
                if current_step not in comprehensively_checked:
                    if options["verbose"]:
                        print("Performing comprehensive IK check for step {}".format(current_step))
                    checking_ik_options = deepcopy(ik_options)
                    checking_ik_options["max_random_restart"] = 10
                    checking_ik_generator = planner._iter_inverse_kinematics_point_axis_target(
                        all_targets[current_step], intermediate_state, group, ik_options
                    )
                    try:
                        next(checking_ik_generator, None)
                    except InverseKinematicsError:
                        raise MPNoIKSolutionError(
                            "No IK solution found for step {} of {}".format(current_step, len(all_targets) - 1),
                            all_targets[current_step],
                            _build_return_trajectory(planned_configurations),
                        )
                    comprehensively_checked.append(current_step)

            # Check IK result for max_jump
            if ik_result is not None:
                try:
                    check_max_jump(
                        joint_names,
                        joint_types,
                        planned_configurations[-1].joint_values,
                        ik_result.joint_values,
                        options,
                    )
                    within_max_jump = True
                except MPMaxJumpError:
                    within_max_jump = False
                    if options["verbose"]:
                        print("Step: {} Option {} violated max_jump".format(current_step, ik_option_indices[-1]))

            # If valid IK solution found, proceed to the next step
            if ik_result is not None and within_max_jump:

                if options["verbose"]:
                    print("Step: {} Option {} is valid".format(current_step, ik_option_indices[-1]))
                planned_configurations.append(ik_result)

                # If this is the last step, the search is complete
                if len(planned_configurations) == len(all_targets):
                    if options["verbose"]:
                        print("Search is complete. Returning {} configurations".format(len(planned_configurations)))
                    break

                # If this is the longest trajectory found, save it
                if len(planned_configurations) > len(longest_list_of_planned_configurations):
                    longest_list_of_planned_configurations = deepcopy(planned_configurations)

                # Move to the next step
                current_step += 1

            # If no more options and at the current step, backtrack
            else:
                if options["verbose"]:
                    print("Search Exhausted in Step:{} after {} ik options".format(current_step, ik_option_indices[-1]))

                current_step -= 1
                # Stop condition is when the current step is less than 0
                if current_step < 0:
                    message = "No trajectory found for {} interpolated points. The longest planned trajectory covered {} points.".format(
                        len(all_targets), len(longest_list_of_planned_configurations)
                    )
                    if options["verbose"]:
                        print("Search Failed")
                        print(message)
                    longest_trajectory = _build_return_trajectory(longest_list_of_planned_configurations)
                    raise MPNoPlanFoundError(message, longest_trajectory)
                # When backtracking, remove the last planned configuration and the exhausted generator
                ik_generators.pop()
                ik_option_indices.pop()
                planned_configurations.pop()

            # If no valid IK solution, try the next option
            continue

        # ===================================================================================
        # DFS Ends Here
        # ===================================================================================

        return _build_return_trajectory(planned_configurations)

    def plan_cartesian_motion_frame_waypoints(self, waypoints, start_state, group, options=None):
        # type: (FrameWaypoints, RobotCellState, Optional[str], Optional[Dict]) -> JointTrajectory
        """Calculates a cartesian motion path (linear in tool space) for Frame Waypoints.

        See :meth:`~plan_cartesian_motion` for the generic description of the planning function.
        The following description is specific for Frame Waypoints.

        The interpolation for Frame Waypoints is done in Cartesian space, meaning each
        segment (between two target_frames) creates a straight line in Cartesian space.
        The frame being interpolated is specified by the target_mode of the waypoints.

        The planner will attempt to create equally spaced trajectory points between the waypoints.
        The spacing is controlled by the ``max_step_distance`` and ``max_step_angle`` parameters.
        These parameters are useful for controlling the smoothness of the trajectory.

        However, even when two points are close in Cartesian space,
        their joint positions may still be far apart.
        This can be dangerous for the robot, as it may cause sudden high-speed jumps in
        joint space, and large gaps in the collision checking process.
        To prevent this, the planner will check whether the jump between each consecutive
        joint position is within the maximum allowed distance specified by the
        ``max_jump_prismatic`` and ``max_jump_revolute`` parameters.
        If the jump is too large, the planner will subdivide the interpolation until the
        jump is within the limits.
        When this happens, the spacing between the interpolated points will no longer be equal.

        The subdivision process have a maximum limit to prevent the planner from
        subdividing indefinitely, such as the case near singularities.
        This is controlled by the ``min_step_distance`` and ``min_step_angle`` parameters.
        Typically, this is set to a small value 8 to 16 times smaller than the ``max_step`` parameters.
        When the interpolation cannot be subdivided further, it is an indication that the robot
        cannot reach the next point without making a large jump in joint space.
        In this case, the planner will raise a JointJumpError and stop the planning process.

        The following parameters are used to control the interpolation process.
        The interpolator will respect these parameters and will not exceed them,
        if the interpolation cannot be done within these limits,
        the planner will raise an error and stop the planning process.

        Notes
        -----

        Users should call :meth:`~PyBulletPlanCartesianMotion.plan_cartesian_motion` instead of this method directly.
        Doing so will ensure that necessary scaling and checks are performed.

        This planning function is deterministic, meaning that the same input will always result in the same output.
        If this function fails, retrying with the same input will not change the result.
        When planning fails, the user can still obtain the partial trajectory by

        The planner will not attempt to avoid collision even when ``check_collision`` is enabled.
        It simply check for them and raise an error if a collision is found.

        This planning function is not asynchronous, meaning that it will block the main thread until the planning is complete
        or an error is raised.

        Parameters
        ----------
        waypoints : :class:`compas_fab.robots.FrameWaypoints`
            The waypoints for the robot to follow.
        start_state : :class:`compas_fab.robots.RobotCellState`
            The starting state of the robot cell at the beginning of the motion.
            The attribute `robot_configuration`, must be provided.
        group: str
            The planning group used for calculation.
        options: dict, optional
            Dictionary containing the following key-value pairs:

            - ``"max_step_distance"``: (:obj:`float`, optional)
              The max Cartesian distance between two consecutive points in the result.
              Unit is in meters, defaults to ``0.01``.
            - ``"max_step_angle"``: (:obj:`float`, optional)
              The max angular distance between two consecutive points in the result.
              Unit is in radians, defaults to ``0.1``.
            - ``"min_step_distance"``: (:obj:`float`, optional)
              The min Cartesian distance between two consecutive points
              when the interpolation is subdivided.
              Unit is in meters, defaults to ``0.0001``.
            - ``"min_step_angle"``: (:obj:`float`, optional)
              The min angular distance between two consecutive points
              when the interpolation is subdivided.
              Unit is in radians, defaults to ``0.0001``.
            - ``"max_jump_prismatic"``: (:obj:`float`, optional)
              The maximum allowed distance of prismatic joint positions
              between consecutive trajectory points.
              Unit is in meters, defaults to ``0.1``.
              Setting this to ``0`` will disable this check.
            - ``"max_jump_revolute"``: (:obj:`float`, optional)
              The maximum allowed distance of revolute and continuous joint positions
              between consecutive trajectory points.
              Unit is in radians, defaults to :math:`\\pi / 2` (90 degrees).
              Setting this to ``0`` will disable this check.
            - ``"check_collision"``: (:obj:`bool`, optional)
              Whether or not to avoid collision. Defaults to ``True``.
            - ``"verbose"``: (:obj:`bool`, optional)
                Whether or not to print verbose output. Defaults to ``False``.
            - ``"skip_preplanning_collision_check"``: (:obj:`bool`, optional)
                Whether or not to skip the target check before planning. Defaults to ``False``.
                The preplanning check in intended to provide a fail-fast feedback for the user.
                However, if there are many targets in the waypoints, the check maybe more
                time consuming than the benefit it provides.

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
            The calculated trajectory.

        Raises
        ------
        :class:`compas_fab.backends.MPInterpolationInCollisionError`
            If ``check_collision`` is enabled and the configuration is in collision.
            The partially planned trajectory before the collision is returned.
            The offending target and collision pairs are also returned.
        :class:`compas_fab.backends.MPNoIKSolutionError`
            If no IK solution could be found by the kinematic solver.
            The partially planned trajectory before the unreachable target is returned.
            The unreachable target frame is returned.
        :class:`compas_fab.backends.MPMaxJumpError`
            If the joint positions between two consecutive points exceed the maximum allowed distance
            and the interpolation cannot be subdivided further due to the ``min_step_*`` parameter.
            The joint name, type, values, and the difference are returned.
        :class:`compas_fab.backends.MPStartStateInCollisionError`
            If the start state is in collision.
            This is part of a sanity check before the planning process.


        """
        # Set default option values
        options = options or {}
        options = deepcopy(options)
        options["max_step_distance"] = options.get("max_step_distance", 0.01)  # meters
        options["max_step_angle"] = options.get("max_step_angle", 0.1)  # radians
        options["min_step_distance"] = options.get("min_step_distance", 0.0001)  # meters
        options["min_step_angle"] = options.get("min_step_angle", 0.0001)  # radians
        options["max_jump_prismatic"] = options.get("max_jump_prismatic", 0.1)
        options["max_jump_revolute"] = options.get("max_jump_revolute", pi / 2)
        options["check_collision"] = options.get("check_collision", True)
        options["skip_preplanning_collision_check"] = options.get("skip_preplanning_collision_check", False)
        options["verbose"] = options.get("verbose", False)

        # Housekeeping for intellisense
        planner = self  # type: PyBulletPlanner
        client = planner.client  # type: PyBulletClient
        robot_cell = client.robot_cell  # type: RobotCell

        # Setting robot cell state
        planner.set_robot_cell_state(start_state)

        # Check start state input for collision if requested, it will throw an exception if the robot is in collision
        if options.get("check_collision"):
            # This allows options such as `full_report` and `verbose` to pass through to the check_collision method
            options.update({"_skip_set_robot_cell_state": True})

            # Check if the start state is in collision
            try:
                # Note: This is using the CheckCollision Backend Feature
                planner.check_collision(start_state, options)
            except CollisionCheckError as e:
                message = (
                    "plan_cartesian_motion_frame_waypoints: The start_state for plan_cartesian_motion is in collision. \n  - "
                    + e.message
                )
                raise MPStartStateInCollisionError(message, start_state=start_state, collision_pairs=e.collision_pairs)

        # Checking the attached tool and workpiece for collision at every target
        if options.get("check_collision") and not options.get("skip_preplanning_collision_check"):
            intermediate_state = deepcopy(start_state)
            intermediate_state.robot_configuration = None
            # Convert the targets to PCFs for collision checking
            pcf_frames = client.robot_cell.target_frames_to_pcf(
                start_state, waypoints.target_frames, waypoints.target_mode, group
            )

            for pcf_frame in pcf_frames:

                try:
                    planner.check_collision_for_attached_objects_in_planning_group(
                        intermediate_state,
                        group,
                        pcf_frame,
                        options,
                    )
                except CollisionCheckError as e:
                    message = (
                        "plan_cartesian_motion_frame_waypoints: The target frame for plan_cartesian_motion is in collision. \n  - "
                        + e.message
                    )
                    raise MPTargetInCollisionError(message, target=pcf_frame, collision_pairs=e.collision_pairs)

        # Options for Inverse Kinematics
        ik_options = deepcopy(options)
        # NOTE: PyBullet IK is gradient based and will snap to the nearest configuration of the start state
        # NOTE: We are running this IK solver without random search (by setting max_results = 1) to ensure determinism
        #       In this mode, planner.inverse_kinematics() will raise an InverseKinematicsError if no IK solution is found,
        #       or CollisionCheckError if the configuration is found but in collision. We will handle these errors below.
        ik_options["max_results"] = 1
        # NOTE: # We only need the joint values for the group to construct the JointTrajectoryPoint
        ik_options["return_full_configuration"] = False
        # NOTE: The PyBullet IK function is responsible for performing collision checking for this planning function
        # The ["check_collision"] in options is passed also to the ik_options

        # Getting the joint names this way ensures that the joint order is consistent with Semantics
        joint_names = robot_cell.get_configurable_joint_names(group)
        joint_types = robot_cell.get_configurable_joint_types(group)

        # Iterate over the waypoints as segments
        intermediate_state = deepcopy(start_state)  # type: RobotCellState
        start_configuration = start_state.robot_configuration
        # TODO: We currently trust that the input configuration has a correct joint order, this should be checked
        trajectory = JointTrajectory(joint_names=joint_names, start_configuration=start_configuration)

        # Add the start configuration as the first point
        joint_values = [start_configuration[joint_name] for joint_name in joint_names]
        trajectory.points.append(
            JointTrajectoryPoint(joint_values=joint_values, joint_types=joint_types, joint_names=joint_names)
        )

        # Echo target_mode
        if options["verbose"]:
            print("Target Mode is: {}. Interpolation will happen with this reference".format(waypoints.target_mode))

        # Recreate the first frame for the beginning of the interpolation
        # First frame is obtained from the start configuration with forward kinematics
        fk_options = deepcopy(options)
        fk_options["link"] = robot_cell.get_end_effector_link_name(group)
        # This frame reference need to match with the target_mode of the waypoints
        start_frame = planner.forward_kinematics(start_state, waypoints.target_mode, group=group, options=fk_options)

        if options["verbose"]:
            print("Start frame: {}".format(start_frame))

        # Begin the interpolation, i iterates over the segments, j iterates over the interpolation steps
        # NOTE: The interpolation happens with the target_frames (in which ever reference they are specified by waypoint.target_mode)
        # NOTE: The PyBullet IK function is responsible for converting the target_frames to PCF and solve for that.
        for i in range(len(waypoints.target_frames)):
            # Calculate interpolation steps based on distance and angle
            # NOTE: Start frame for this segment is the end frame of the previous segment
            end_frame = waypoints.target_frames[i]  # type: Frame

            # Create interpolation helper object
            interpolator = FrameInterpolator(start_frame, end_frame, options)

            if options["verbose"]:
                print(
                    "Segment {} of {}, Interpolating between {} and {}, distance={} requiring {} steps.".format(
                        i + 1,
                        len(waypoints.target_frames),
                        start_frame,
                        end_frame,
                        interpolator.total_distance,
                        interpolator.regular_interpolation_steps,
                    )
                )

            # Compute a list of t values for the interpolation (0 <= t <= 1)
            # Values 0 is the start frame and 1 is the end frame
            # Intermediary values can be added during interpolation if joint jump is too large
            interpolation_ts = []
            # The first number is zero, which correspond to the starting config
            # The first IK will skip this value and start from the second value (where j = 1)
            for j in range(interpolator.regular_interpolation_steps + 1):
                interpolation_ts.append(j / interpolator.regular_interpolation_steps)

            # Iterative IK with sub-division of the interpolation
            j = 1
            while j < len(interpolation_ts):
                # Compute the interpolated frame
                current_frame = interpolator.get_interpolated_frame(interpolation_ts[j])

                # Prepare the Intermediate State and Target for IK
                target = FrameTarget(
                    current_frame,
                    target_mode=waypoints.target_mode,
                    tolerance_position=waypoints.tolerance_orientation,
                    tolerance_orientation=waypoints.tolerance_orientation,
                )
                intermediate_state.robot_configuration.joint_values = trajectory.points[-1].joint_values
                if options["verbose"]:
                    print(
                        "Segment {} of {}, j={}, t = {}, Interpolated Frame = {}".format(
                            i + 1, len(waypoints.target_frames), j, interpolation_ts[j], current_frame
                        )
                    )

                # Call Inverse Kinematics function from planner
                try:
                    # Try block to catch InverseKinematicsError, if IK failed, planning is stopped
                    configuration = planner.inverse_kinematics(target, intermediate_state, group, ik_options)
                    new_joint_positions = [configuration[joint_name] for joint_name in joint_names]

                # Catch the InverseKinematicsError and CollisionCheckError and re-raise them with additional information
                except InverseKinematicsError as e:
                    message = "plan_cartesian_motion_frame_waypoints(): Segment {}, Inverse Kinematics failed at t={}.\n".format(
                        i, interpolation_ts[j]
                    )
                    message = message + e.message
                    raise MPNoIKSolutionError(message=message, target=target, partial_trajectory=trajectory)

                except CollisionCheckError as e:
                    message = "plan_cartesian_motion_frame_waypoints(): Segment {}, Inverse Kinematics failed at t={}.\n".format(
                        i, interpolation_ts[j]
                    )
                    message = message + e.message
                    raise MPInterpolationInCollisionError(
                        message=message, target=target, collision_pairs=e.collision_pairs, partial_trajectory=trajectory
                    )
                if options["verbose"]:
                    print(
                        "Segment {} of {}, j={}, t={}, joint_values={}".format(
                            i + 1, len(waypoints.target_frames), j, interpolation_ts[j], new_joint_positions
                        )
                    )

                # Check `joint_jump` between the current and previous point's configuration
                try:
                    check_max_jump(
                        joint_names, joint_types, trajectory.points[-1].joint_values, new_joint_positions, options
                    )
                except MPMaxJumpError as e:
                    # Check if further subdivision is possible
                    delta_distance, delta_angle, subdivision_possible = interpolator.check_if_subdivision_possible(
                        interpolation_ts[j - 1], interpolation_ts[j]
                    )

                    #  If it is not possible to subdivide, raise an error and stop planning
                    if not subdivision_possible:
                        # Raise the error with additional information about the interpolation
                        message = "plan_cartesian_motion_frame_waypoints(): Segment {} of {}, Joint jump between t={} and t={} is too large.\n  -  {}".format(
                            i + 1, len(waypoints.target_frames), interpolation_ts[j - 1], interpolation_ts[j], e.message
                        )
                        message += "\nplan_cartesian_motion_frame_waypoints(): Cannot subdivide further between t={} and t={}, current delta_distance={} (limit={}), delta_angle={} (limit={}).".format(
                            interpolation_ts[j - 1],
                            interpolation_ts[j],
                            delta_distance,
                            options["min_step_distance"],
                            delta_angle,
                            options["min_step_angle"],
                        )
                        e.message = message
                        raise e

                    # If it is possible to subdivide, we insert a new t value in the middle
                    subdivided_t = interpolation_ts[j - 1] + (interpolation_ts[j] - interpolation_ts[j - 1]) / 2
                    interpolation_ts.insert(j, subdivided_t)
                    if options["verbose"]:
                        print(
                            "Segment {} of {}, j={}, subdivision added due to joint jump, new t={:.5f} .".format(
                                i + 1, len(waypoints.target_frames), j, subdivided_t
                            )
                        )
                        print()
                    # Try again with the new t value
                    continue

                # This point is successful, add the JointTrajectoryPoint to the trajectory
                trajectory.points.append(
                    JointTrajectoryPoint(
                        joint_values=new_joint_positions, joint_types=joint_types, joint_names=joint_names
                    )
                )
                if options["verbose"]:
                    print(
                        "Segment {} of {}, JointTrajectoryPoint [{}] added: {}".format(
                            i + 1, len(waypoints.target_frames), j, new_joint_positions
                        )
                    )
                if options.get("step_pause"):
                    input("Press Enter to continue...")
                j += 1

            if options["verbose"]:
                print("Segment {} of {} completed.".format(i + 1, len(waypoints.target_frames)))
            # Set the start frame for the next segment
            start_frame = end_frame

        return trajectory


class FrameInterpolator(object):
    """A helper class to interpolate between two frames

    All the properties are read-only and are computed during initialization.

    Parameters
    ----------
    start_frame : :class:`compas.geometry.Frame`
        The start frame.
    end_frame : :class:`compas.geometry.Frame`
        The end frame.
    options : dict
        Dictionary should contain the following key-value pairs:

        - ``"max_step_distance"``: (:obj:`float`, optional)
        - ``"max_step_angle"``: (:obj:`float`, optional)
        - ``"min_step_distance"``: (:obj:`float`, optional)
        - ``"min_step_angle"```: (:obj:`float`, optional)

    """

    def __init__(self, start_frame, end_frame, options):
        # type: (Frame, Frame, Dict) -> None
        self.start_frame = start_frame
        self.end_frame = end_frame
        self.options = options

        # Compute the total distance between the two frames
        self._total_distance = self.start_frame.point.distance_to_point(self.end_frame.point)

        # Compute the total angle between the two frames
        delta_frame = self.start_frame.to_local_coordinates(self.end_frame)
        _, self._total_angle = axis_angle_from_quaternion(Quaternion.from_frame(delta_frame))

        # Compute the number of steps based on max_step_distance and max_step_angle
        num_steps_by_distance = ceil(self._total_distance / self.options["max_step_distance"])
        num_steps_by_angle = ceil(self._total_angle / self.options["max_step_angle"])
        # NOTE: Minimum of 1 step is used, this step is equal to the end_frame itself.
        self._regular_interpolation_steps = max(num_steps_by_distance, num_steps_by_angle, 1)

    @property
    def total_distance(self):
        # type: () -> float
        """The total distance between the start and end frames.

        Returns
        -------
        float
            The total distance (original unit) between the start and end frames.

        """
        return self._total_distance

    @property
    def total_angle(self):
        # type: () -> float
        """Tte total angle between the start and end frames.

        Returns
        -------
        float
            The total angle in radians between the start and end frames.

        """
        return self._total_angle

    @property
    def regular_interpolation_steps(self):
        # type: () -> int
        """The number of interpolation steps based on max_step_distance and max_step_angle.

        Returns
        -------
        int
            The number of interpolation steps.
            Minimum is 1.

        """

        return self._regular_interpolation_steps

    def get_interpolated_frame(self, t):
        # type: (float) -> Frame
        """Interpolate between two frames using a parameter t.

        Parameters
        ----------
        t : float
            The interpolation parameter, 0 <= t <= 1.

        Returns
        -------
        :class:`compas.geometry.Frame`
            The interpolated frame.

        """
        # Compute the interpolated frame
        current_frame = self.start_frame.interpolate_frame(self.end_frame, t)
        return current_frame

    def check_if_subdivision_possible(self, t1, t2):
        # type: (float, float) -> bool
        """Check if the addition of an extra t value between t1 and t2 is possible.

        Two conditions are being checked:

        1. The distance between the two frames is greater than the `min_step_distance`.
        2. The angle between the two frames is greater than the `min_step_angle`.

        The subdivision is possible if any one of the two conditions is met.

        Parameters
        ----------
        t1 : float
            The start parameter.
        t2 : float
            The end parameter.

        Returns
        -------
        tuple
            A tuple containing the following values:

            - ``delta_distance``: (:obj:`float`) The distance between the two frames.
            - ``delta_angle``: (:obj:`float`) The angle between the two frames.
            - ``subdivision_possible``: (:obj:`bool`) True if the subdivision is possible
        """
        delta_t = abs(t2 - t1)

        delta_distance = self.total_distance * delta_t
        delta_angle = self.total_angle * delta_t

        if delta_distance < self.options["min_step_distance"] * 2 and delta_angle < self.options["min_step_angle"] * 2:
            return (delta_distance, delta_angle, False)

        return (delta_distance, delta_angle, True)


class PointAxisInterpolator(object):
    """A helper class to interpolate between two point-axis pairs.

    All the properties are read-only and are computed during initialization.

    Parameters
    ----------
    start_point_axis : tuple of :class:`compas.geometry.Point` and :class:`compas.geometry.Vector`
        The start point-axis pair.
    end_point_axis : tuple of :class:`compas.geometry.Point` and :class:`compas.geometry.Vector`
        The end point-axis pair.
    options : dict
        Dictionary should contain the following key-value pairs:

        - ``"max_step_distance"``: (:obj:`float`, optional)
        - ``"max_step_angle"``: (:obj:`float`, optional)
        - ``"min_step_distance"``: (:obj:`float`, optional)
        - ``"min_step_angle"```: (:obj:`float`, optional)

    """

    def __init__(self, start_point_axis, end_point_axis, options):
        # type: (Tuple[Point, Vector], Tuple[Point, Vector], Dict) -> None
        self.start_point, self.start_axis = start_point_axis
        self.end_point, self.end_axis = end_point_axis
        self.options = options

        # Compute the total distance between the two points
        self._total_distance = self.start_point.distance_to_point(self.end_point)

        # Compute the total angle between the two axes
        self._total_angle = self.start_axis.angle(self.end_axis)

        # Compute the number of steps based on max_step_distance and max_step_angle
        num_steps_by_distance = ceil(self._total_distance / self.options["max_step_distance"])
        num_steps_by_angle = ceil(self._total_angle / self.options["max_step_angle"])

        # NOTE: Minimum of 1 step is used, this step is equal to the end_frame itself.
        self._regular_interpolation_steps = max(num_steps_by_distance, num_steps_by_angle, 1)

    @property
    def total_distance(self):
        # type: () -> float
        """The total distance between the start and end points.

        Returns
        -------
        float
            The total distance (original unit) between the start and end points.

        """
        return self._total_distance

    @property
    def total_angle(self):
        # type: () -> float
        """The total angle between the start and end axes.

        Returns
        -------
        float
            The total angle in radians between the start and end axes.

        """
        return self._total_angle

    @property
    def regular_interpolation_steps(self):
        # type: () -> int
        """The number of interpolation steps based on max_step_distance and max_step_angle.

        Returns
        -------
        int
            The number of interpolation steps.
            Minimum is 1.

        """

        return self._regular_interpolation_steps

    def get_interpolated_point_axis(self, t):
        # type: (float) -> Tuple[List[float], Vector]
        """Interpolate between two point-axis pairs using a parameter t.

        Parameters
        ----------
        t : float
            The interpolation parameter, 0 <= t <= 1.

        Returns
        -------
        tuple of :class:`compas.geometry.Point` and :class:`compas.geometry.Vector`
            The interpolated point-axis pair.

        """
        # Compute the interpolated point using the parameter t
        xyz = [a + t * (b - a) for a, b in zip(self.start_point, self.end_point)]
        point_t = xyz
        # Compute the interpolated axis by rotation
        if is_parallel_vector_vector(self.start_axis, self.end_axis):
            axis_t = self.end_axis
            # print("Parallel")
        else:
            angle = t * self.total_angle
            rotation_axis = cross_vectors(self.start_axis, self.end_axis)
            axis_t = self.start_axis.rotated(angle, axis=rotation_axis)
            # print("Not Parallel, rotated by {} degrees".format(angle))

        print("Interpolated t = {}, Point={}, Axis={}".format(t, point_t, axis_t))
        return (point_t, axis_t)
