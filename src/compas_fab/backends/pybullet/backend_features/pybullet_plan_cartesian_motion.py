from compas_fab.robots import Waypoints
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import PointAxisWaypoints
from compas_fab.robots import FrameTarget
from compas_fab.robots import JointTrajectory
from compas_fab.robots import JointTrajectoryPoint
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
        If tools are attached to the robot, it must be reflected using the starting state.
        In this case, the waypoints are describing the robot's Tool Coordinate Frame (TCF)
        instead of the Planner Coordinate Frame (PCF) of the robot.

        The robot's full configuration, i.e. values for all configurable joints of the entire robot,
        must be provided in the ``start_state.robot_configuration`` parameter.
        The ``start_state.robot_flange_frame`` parameter is not used by the planning function.

        The ``waypoints`` parameter is used to defined single or multiple segments of path, it is not necessary to include
        the starting pose in the waypoints, doing so may result in a redundant trajectory point.
        The waypoints refers to the robot's Tool0 Coordinate Frame (T0CF) when no tools are attached,
        or the Tool's Coordinate Frame (TCF) when a tool is attached.

        Each path segment is interpolated linearly in Cartesian space, where position is interpolated linearly in Cartesian space
        and orientation is interpolated spherically using quaternion interpolation.

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



        """

        # NOTE: The conversion of Waypoints Targets from TCF to T0CF must happen inside the
        # corresponding planner method. This is because for PointAxisWaypoints, the planner
        # must be able to spin around the TCF Z axis freely.
        # This is not possible anymore if the targets are converted to T0CF representation here.

        # Unit conversion from user scale to meter scale can be done here because they are shared.
        robot = self.client.robot  # type: Robot
        if robot.need_scaling:
            waypoints = waypoints.scaled(1.0 / robot.scale_factor)

        if isinstance(waypoints, PointAxisWaypoints):
            return self.plan_cartesian_motion_point_axis_waypoints(waypoints, start_state, group, options)
        elif isinstance(waypoints, FrameWaypoints):
            return self.plan_cartesian_motion_frame_waypoints(waypoints, start_state, group, options)
        else:
            raise NotImplementedError("Only PointAxisWaypoints and FrameWaypoints are supported.")

    def plan_cartesian_motion_point_axis_waypoints(self, waypoints, start_state, group=None, options=None):
        # type: (PointAxisWaypoints, RobotCellState, Optional[str], Optional[Dict]) -> JointTrajectory
        """Calculates a cartesian motion path (linear in tool space) for Point Axis Waypoints."""

        raise NotImplementedError("plan_cartesian_motion_point_axis_waypoints() is not implemented yet.")

    def plan_cartesian_motion_frame_waypoints(self, waypoints, start_state, group=None, options=None):
        # type: (FrameWaypoints, RobotCellState, Optional[str], Optional[Dict]) -> JointTrajectory
        """Calculates a cartesian motion path (linear in tool space) for Frame Waypoints.

        The ``waypoints`` parameter is used to defined single or multiple segments of path, it is not necessary to include
        the starting pose in the waypoints, doing so may result in a redundant trajectory point.
        The waypoints refers to the robot's Tool0 Coordinate Frame (T0CF) when no tools are attached,
        or the Tool's Coordinate Frame (TCF) when a tool is attached.
        Each path segment is interpolated linearly in Cartesian space, where position is interpolated linearly in Cartesian space
        and orientation is interpolated spherically using quaternion interpolation.

        If the planning is successful, the function will return a :class:`compas_fab.robots.JointTrajectory` object,
        which contains the start_configuration (same as input) and a list of :class:`compas_fab.robots.JointTrajectoryPoint`.

        - Note that the first point in the trajectory will not repeat the start_configuration.
        - Note that there may be one or more trajectory point for each path segment due to interpolation.
        - Note that the returned trajectory does not separate the trajectory points into segments based on the waypoints.
          If the user wants to separate the path into segments, they should consider calling this function multiple times.

        The ``check_collision`` parameter can be used to enable or disable collision checking during the planning process.
        If enabled, the planner will check for collision similar to the :meth:`compas_fab.backends.PyBulletClient.check_collision` method.

        The following parameters are used to control the interpolation process.
        The interpolator will respect these parameters and will not exceed them, if the interpolation cannot be done within these limits,
        the planner will raise a JointJumpError.

        - The ``max_step_distance`` and ``max_step_angle`` parameters, controls the amount of subdivision by limiting the distance and angle
          between interpolated points (measured at TCF or PCF).
        - The ``max_jump_prismatic`` and ``max_jump_revolute`` parameters are used to control the maximum allowed joint distance
          between consecutive points. This can prevent the robot from making sudden high speed jumps in joint space.
          This can also avoid large gaps in the collision checking process.
          Units are in meters for prismatic joints and radians for revolute and continuous joints.
        - The ``min_step_distance`` and ``min_step_angle`` parameter prevents the planner from subdividing
          the interpolation excessively due to the ``max_jump`` parameters.


        Notes
        -----

        Trajectory points are not guaranteed to be equidistant in Cartesian or Joint space.

        This planning function is deterministic, meaning that the same input will always result in the same output.
        If this function fails, retrying with the same input will not change the result.

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

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
            The calculated trajectory.

        Raises
        ------
        CollisionCheckError
            If ``check_collision`` is enabled and the configuration is in collision.
        InverseKinematicsError
            If no IK solution could be found by the kinematic solver.
            The target frame may be unreachable.
        JointJumpError
            If the joint positions between two consecutive points exceed the maximum allowed distance
            and the interpolation cannot be subdivided further due to the ``min_step_*`` parameter.

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
        group = group or robot.main_group_name

        # Setting robot cell state
        planner.set_robot_cell_state(start_state)

        # Check start state input for collision if requested, it will throw an exception if the robot is in collision
        if options.get("check_collision"):
            # This allows options such as `full_report` and `verbose` to pass through to the check_collision method
            options.update({"_skip_set_robot_cell_state": True})
            try:
                # Note: This is using the CheckCollision Backend Feature
                planner.check_collision(start_state, options)
            except CollisionCheckError as e:
                message = (
                    "plan_cartesian_motion_frame_waypoints: The start_state for plan_cartesian_motion is in collision. \n  - "
                    + e.message
                )
                raise MPStartStateInCollisionError(message, start_state=start_state, collision_pairs=e.collision_pairs)

        # Options for Inverse Kinematics
        # max_results = 1 removes the random search of the IK Engine
        options["max_results"] = options.get("max_results", 1)
        options["return_full_configuration"] = False  # We only need the joint values for the group

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
        # First frame is obtained from the start configuration with forward kinematics
        start_frame = planner.forward_kinematics(start_state, group, options)  # type: Frame
        if options["verbose"]:
            print("Start frame: {}".format(start_frame))
        for i in range(len(waypoints.target_frames)):
            # Calculate interpolation steps based on distance and angle
            # Start frame is the end frame of the previous segment
            end_frame = waypoints.target_frames[i]  # type: Frame
            interpolation_total_distance = start_frame.point.distance_to_point(end_frame.point)
            delta_frame = start_frame.to_local_coordinates(end_frame)
            _, interpolation_total_angle = axis_angle_from_quaternion(Quaternion.from_frame(delta_frame))
            steps_by_distance = ceil(interpolation_total_distance / options["max_step_distance"])
            steps_by_angle = ceil(interpolation_total_angle / options["max_step_angle"])
            interpolation_steps = max(steps_by_distance, steps_by_angle, 1)
            if options["verbose"]:
                print(
                    "Segment {} of {}, Interpolating {} steps between {} and {}, distance={} requiring {} steps.".format(
                        i + 1,
                        len(waypoints.target_frames),
                        interpolation_steps,
                        start_frame,
                        end_frame,
                        interpolation_total_distance,
                        interpolation_steps,
                    )
                )

            # Compute a list of t values for the interpolation (0 <= t <= 1)
            # Values 0 is the start frame and 1 is the end frame
            # Intermediary values can be added during interpolation if joint jump is too large
            interpolation_ts = []
            # The first number is zero, which correspond to the starting config
            for j in range(interpolation_steps + 1):
                interpolation_ts.append(j / interpolation_steps)

            # Iterative IK with sub-division of the interpolation
            j = 1
            while j < len(interpolation_ts):
                # Compute the interpolated frame
                current_frame = start_frame.interpolate_frame(end_frame, interpolation_ts[j])
                # Prepare the input for IK
                target = FrameTarget(current_frame)
                intermediate_state.robot_configuration.joint_values = trajectory.points[-1].joint_values
                # Note that the PyBullet IK also performs collision checking
                # Note that the PyBullet IK is gradient based and will snap to the nearest configuration of the start state
                # Note that we are running this IK solver without random search (max_results = 1) to ensure determinism
                # In this mode, planner.inverse_kinematics() will raise an InverseKinematicsError if no IK solution is found,
                # or CollisionCheckError if the configuration is found but in collision.
                if options["verbose"]:
                    print(
                        "Segment {} of {}, j={}, t = {}, Interpolated Frame = {}".format(
                            i + 1, len(waypoints.target_frames), j, interpolation_ts[j], current_frame
                        )
                    )

                # Perform Inverse Kinematics
                try:
                    # Try block to catch InverseKinematicsError, if IK failed, planning is stopped
                    configuration = planner.inverse_kinematics(target, intermediate_state, group, options)
                    new_joint_positions = [configuration[joint_name] for joint_name in joint_names]
                except InverseKinematicsError as e:
                    message = "plan_cartesian_motion_frame_waypoints(): Segment {}, Inverse Kinematics failed at t={}.\n".format(
                        i, interpolation_ts[j]
                    )
                    message = message + e.message
                    raise MPNoIKSolutionError(message=message, target=target)

                except CollisionCheckError as e:
                    message = "plan_cartesian_motion_frame_waypoints(): Segment {}, Inverse Kinematics failed at t={}.\n".format(
                        i, interpolation_ts[j]
                    )
                    message = message + e.message
                    raise MPInterpolationInCollisionError(
                        message=message, target=target, collision_pairs=e.collision_pairs
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
                    delta_t = interpolation_ts[j] - interpolation_ts[j - 1]
                    delta_distance = interpolation_total_distance * delta_t
                    delta_angle = interpolation_total_angle * delta_t
                    #  If it is not possible to subdivide, raise an error and stop planning
                    if (
                        delta_distance < options["min_step_distance"] * 2
                        and delta_angle < options["min_step_angle"] * 2
                    ):
                        # The subdivision is not possible when the current step's distance is less than min_step
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
