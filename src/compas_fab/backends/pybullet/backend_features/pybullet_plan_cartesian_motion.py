from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.robots import Waypoints
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import PointAxisWaypoints
from compas_fab.robots import FrameTarget
from compas_fab.robots import JointTrajectory
from compas_fab.robots import JointTrajectoryPoint
from compas_fab.robots import TargetMode
from compas_robots.model import Joint

from math import pi
from math import ceil

from compas_fab.backends import InverseKinematicsError
from compas_fab.backends import CollisionCheckError
from compas_fab.backends import MPMaxJumpError
from compas_fab.backends import MPNoIKSolutionError
from compas_fab.backends import MPInterpolationInCollisionError
from compas_fab.backends import MPStartStateInCollisionError
from compas_fab.backends import MPTargetInCollisionError


import compas

from compas.geometry import axis_angle_from_quaternion
from compas.geometry import Quaternion

if compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from compas_fab.backends import PyBulletClient  # noqa: F401
        from compas_fab.backends import PyBulletPlanner  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import Robot  # noqa: F401
        from compas_robots import Configuration  # noqa: F401
        from compas.geometry import Frame  # noqa: F401
        from typing import Optional  # noqa: F401
        from typing import Dict  # noqa: F401
        from typing import List  # noqa: F401

__all__ = [
    "PyBulletPlanCartesianMotion",
]
from compas_fab.backends.interfaces import PlanCartesianMotion


class PyBulletPlanCartesianMotion(PlanCartesianMotion):
    """Callable to calculate a cartesian motion path (linear in tool space)."""

    def plan_cartesian_motion(self, waypoints, start_state, group=None, options=None):
        # type: (Waypoints, Configuration, RobotCellState, Optional[Dict]) -> JointTrajectory
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
        The ``start_state.robot_flange_frame`` parameter is not used by the planning function.

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

        # Unit conversion from user scale to meter scale can be done here because they are shared.
        robot = self.client.robot  # type: Robot
        group = group or robot.main_group_name
        if robot.need_scaling:
            waypoints = waypoints.scaled(1.0 / robot.scale_factor)

        # Check if the robot cell state supports the target mode
        planner = self  # type: PyBulletPlanner
        planner.ensure_robot_cell_state_supports_target_mode(start_state, waypoints.target_mode, group)

        # ===================================================================================
        # End of common lines
        # ===================================================================================

        if isinstance(waypoints, PointAxisWaypoints):
            return self.plan_cartesian_motion_point_axis_waypoints(waypoints, start_state, group, options)
        elif isinstance(waypoints, FrameWaypoints):
            return self.plan_cartesian_motion_frame_waypoints(waypoints, start_state, group, options)
        else:
            raise NotImplementedError("Only PointAxisWaypoints and FrameWaypoints are supported.")

    def plan_cartesian_motion_point_axis_waypoints(self, waypoints, start_state, group=None, options=None):
        # type: (PointAxisWaypoints, RobotCellState, Optional[str], Optional[Dict]) -> JointTrajectory
        """Calculates a cartesian motion path (linear in tool space) for Point Axis Waypoints.

        Users can choose to provide a list of high density waypoints or a list of sparse waypoints.
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

        For applications that require a specific tool speed, the user can use the max_jump parameters to ensure that the
        joint speed is within the desired range.

        The planner can be configured to sample the starting configuration to improve the chance of finding a solution,
        this is controlled by the ``sample_start_configuration`` option. If enabled, the planner will treat the starting
        configuration as an freely rotatable point-axis target. If a solution is found, the sampled configuration will be
        returned in trajectory.start_configuration. Users should verify whether the sampled configuration is the same
        as the robot_configuration in the start_state.

        """

        raise NotImplementedError("plan_cartesian_motion_point_axis_waypoints() is not implemented yet.")

    def plan_cartesian_motion_frame_waypoints(self, waypoints, start_state, group=None, options=None):
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
        This planning function is deterministic, meaning that the same input will always result in the same output.
        If this function fails, retrying with the same input will not change the result.
        When planning fails, the user can still obtain the partial trajectory by

        The planner will not attempt to avoid collision even when ``check_collision`` is enabled.
        It simply check for them and raise an error if a collision is found.

        This planning function is not asynchronous, meaning that it will block the main thread until the planning is complete
        or an error is raised.

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
        options = options.copy()
        options["max_step_distance"] = options.get("max_step_distance", 0.01)  # meters
        options["max_step_angle"] = options.get("max_step_angle", 0.1)  # radians
        options["min_step_distance"] = options.get("min_step_distance", 0.0001)  # meters
        options["min_step_angle"] = options.get("min_step_angle", 0.0001)  # radians
        options["max_jump_prismatic"] = options.get("max_jump_prismatic", 0.1)
        options["max_jump_revolute"] = options.get("max_jump_revolute", 3.14 / 2)
        options["check_collision"] = options.get("check_collision", True)
        options["skip_preplanning_collision_check"] = options.get("skip_preplanning_collision_check", False)
        options["verbose"] = options.get("verbose", False)

        # Housekeeping for intellisense
        planner = self  # type: PyBulletPlanner
        client = planner.client  # type: PyBulletClient
        robot = client.robot  # type: Robot

        # Check input sanity
        if not start_state.robot_configuration:
            raise ValueError("start_state.robot_configuration must be provided.")
        if not start_state.robot_configuration.joint_names:
            raise ValueError("start_state.robot_configuration.joint_names must be provided.")
        client.robot_cell.assert_cell_state_match(start_state)

        # Get default group name if not provided
        # Do not skip this line because some functions do not default to main_group_name when group input is None.
        group = group or robot.main_group_name

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
            intermediate_state = start_state.copy()
            intermediate_state.robot_configuration = None
            # Convert the targets to PCFs for collision checking
            pcf_frames = planner.frames_to_pcf(waypoints.target_frames, waypoints.target_mode, group)

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
        ik_options = options.copy()
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
        joint_names = robot.get_configurable_joint_names(group)
        joint_types = robot.get_configurable_joint_types(group)

        # Iterate over the waypoints as segments
        intermediate_state = start_state.copy()  # type: RobotCellState
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

        # First frame is obtained from the start configuration with forward kinematics
        fk_options = options.copy()
        fk_options["link"] = robot.get_end_effector_link_name(group)
        start_frame_pcf = planner.forward_kinematics(start_state, group, fk_options)  # type: Frame
        # This frame reference need to match with the target_mode of the waypoints
        start_frame = None  # type: Frame
        if waypoints.target_mode == TargetMode.TOOL:
            tool_id = start_state.get_attached_tool_id(group)
            start_frame = planner.from_pcf_to_tcf([start_frame_pcf], tool_id)[0]
        elif waypoints.target_mode == TargetMode.WORKPIECE:
            workpiece_id = start_state.get_attached_workpiece_ids(group)[0]
            start_frame = planner.from_pcf_to_ocf([start_frame_pcf], workpiece_id)[0]
        elif waypoints.target_mode == TargetMode.ROBOT:
            start_frame = start_frame_pcf
        else:
            raise NotImplementedError(
                "TargetMode ({}) not supported by PyBulletPlanCartesianMotion".format(waypoints.target_mode)
            )
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
                    self._check_max_jump(
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

    def _check_max_jump(self, joint_names, joint_types, start_joint_values, end_joint_values, options):
        # type: (List[str], List[int], List[float], List[float], Dict) -> None
        """Check if the joint positions between two configurations exceed the maximum allowed distance.


        Parameters
        ----------
        joint_names : list of str
            List of joint names.
        joint_types : list of int
            List of joint types.
        start_joint_values : list of float
            List of joint values for the start configuration.
        end_joint_values : list of float
            List of joint values for the end configuration.
        options : dict
            Dictionary containing the following key-value pairs:

            - ``"max_jump_prismatic"``: (:obj:`float`, optional)
            - ``"max_jump_revolute"``: (:obj:`float`, optional)

            Just pass the options dictionary from the planning function.

        Returns
        -------
        None

        Raises
        ------
        MPMaxJumpError
            If the joint positions between two configurations exceed the maximum allowed
            distance specified in the options dictionary.

        """
        max_jump_prismatic = options.get("max_jump_prismatic", 0.1)
        max_jump_revolute = options.get("max_jump_revolute", 3.14 / 2)

        for joint_name, joint_type, v1, v2 in zip(joint_names, joint_types, start_joint_values, end_joint_values):
            # If the joint is REVOLUTE we check the angular difference:
            if joint_type in [Joint.REVOLUTE]:
                difference = abs(v1 - v2)
                if difference > max_jump_revolute:
                    raise MPMaxJumpError(
                        message="_check_max_jump(): Joint {} (REVOLUTE) jump {} is too large, exceeds 'max_jump_revolute' of {}".format(
                            joint_name, difference, max_jump_revolute
                        ),
                        joint_name=joint_name,
                        joint_type=joint_type,
                        joint_values_a=start_joint_values,
                        joint_values_b=end_joint_values,
                        value_difference=difference,
                        value_threshold=max_jump_revolute,
                    )
            # If the joint is CONTINUOUS the angular difference is calculated in the shortest path:
            elif joint_type in [Joint.CONTINUOUS]:
                diff = abs(v1 - v2) % (2 * pi)  # Normalize the difference to 0 to 2*pi
                if diff > pi:
                    diff = 2 * pi - diff
                if diff > max_jump_revolute:
                    raise MPMaxJumpError(
                        message="_check_max_jump(): Joint {} (CONTINUOUS) jump {} is too large, exceeds 'max_jump_revolute' of {}".format(
                            joint_name, diff, max_jump_revolute
                        ),
                        joint_name=joint_name,
                        joint_type=joint_type,
                        joint_values_a=start_joint_values,
                        joint_values_b=end_joint_values,
                        value_difference=diff,
                        value_threshold=max_jump_revolute,
                    )

            if joint_type in [Joint.PRISMATIC, Joint.PLANAR]:
                diff = abs(v1 - v2)
                if diff > max_jump_prismatic:
                    raise MPMaxJumpError(
                        message="_check_max_jump(): Joint {} (PRISMATIC/PLANAR) jump {} is too large, exceeds 'max_jump_prismatic' of {}".format(
                            joint_name, diff, max_jump_prismatic
                        ),
                        joint_name=joint_name,
                        joint_type=joint_type,
                        joint_values_a=start_joint_values,
                        joint_values_b=end_joint_values,
                        value_difference=diff,
                        value_threshold=max_jump_prismatic,
                    )


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
        """Tthe total angle between the start and end frames.

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
