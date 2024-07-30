from compas_fab.robots import Waypoints
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import PointAxisWaypoints
from compas_fab.robots import FrameTarget

from compas_fab.backends import CollisionCheckInCollisionError
import compas

if compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from compas_fab.backends import PyBulletClient  # noqa: F401
        from compas_fab.backends import PyBulletPlanner  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import JointTrajectory  # noqa: F401
        from compas_fab.robots import JointTrajectoryPoint  # noqa: F401
        from compas_fab.robots import Robot  # noqa: F401
        from compas_robots import Configuration  # noqa: F401
        from compas.geometry import Frame  # noqa: F401
        from typing import Optional  # noqa: F401
        from typing import Dict  # noqa: F401

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

        For more information such as planner specific behaviors, see
        :meth:`~compas_fab.backends.PyBulletClient.plan_cartesian_motion_point_axis_waypoints` for
        :class:`~compas_fab.robots.PointAxisWaypoints` and
        :meth:`~compas_fab.backends.PyBulletClient.plan_cartesian_motion_frame_waypoints` for
        :class:`~compas_fab.robots.FrameWaypoints`.

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
        CollisionCheckError
            If ``check_collision`` is enabled and the configuration is in collision.
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

        The starting state of the robot cell at the beginning of the motion must be provided.
        It must match the objects in the robot cell previously set using :meth:`compas_fab.backends.PyBulletClient.set_robot_cell`.
        If tools are attached to the robot, it must be reflected in the starting state.
        The robot's full configuration, i.e. values for all configurable joints of the entire robot,
        must be provided using the ``start_state.robot_configuration`` parameter.
        The ``start_state.robot_flange_frame`` parameter is not used by this function.

        The ``waypoints`` parameter is used to defined single or multiple segments of path, it is not necessary to include
        the starting pose in the waypoints, doing so may result in a redundant trajectory point.
        The waypoints refers to the robot's Tool0 Coordinate Frame (T0CF) if no tools are attached,
        or Tool Coordinate Frame (TCF) if a tool is attached.
        Each segment is interpolated linearly in Cartesian space, where position is interpolated linearly in Cartesian space
        and orientation is interpolated spherically using quaternion interpolation.

        The function will return a :class:`compas_fab.robots.JointTrajectory` object,
        which contains a series of joint trajectory points.
        Note that there maybe more than one trajectory point per path segment due to interpolation.

        The ``check_collision`` parameter can be used to enable or disable collision checking during the planning process.
        If enabled, the planner will check for collision similar to the :meth:`compas_fab.backends.PyBulletClient.check_collision` method.

        There are three parameters that can be used to control the interpolation process, all of which control
        the maximum allowed distance between consecutive points in the result.
        The ``max_step`` parameter, controls the Cartesian distance between the interpolated points (TCF or T0CF).
        The ``max_jump_prismatic`` and ``max_jump_revolute`` parameter, can be used to control the maximum allowed joint distance
        between consecutive points. Units are in meters for prismatic joints and radians for revolute and continuous joints.
        The interpolator will respect these parameters and will not exceed them, however, this also means that the resulting
        trajectory points may not be equidistant in Cartesian or Joint space.

        Note that the resulting trajectory does not separate the trajectory points into segments based on the waypoints.
        If the user wants to separate the path into segments, they should consider calling this function multiple times.

        Notes
        -----

        This planning function is deterministic, meaning that the same input will always result in the same output.
        If this function fails, retrying with the same input will not change the result.

        The planner will not attempt to avoid collision, but simply check for them and raise an error if a collision is found.

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

            - ``"max_step"``: (:obj:`float`, optional)
              The max Cartesian distance between two consecutive points in the result.
              calculated points.
              Unit is in meters, defaults to ``0.01``.
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

        """
        # Set default option values
        options = options or {}
        options["max_step"] = options.get("max_step", 0.01)  # meters
        options["max_jump_prismatic"] = options.get("max_jump_prismatic", 0.1)
        options["max_jump_revolute"] = options.get("max_jump_revolute", 3.14 / 2)
        options["check_collision"] = options.get("check_collision", True)

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
            cc_options = options.copy()
            cc_options.update({"_skip_set_robot_cell_state": True})
            try:
                # Note: This is using the CheckCollision Backend Feature
                planner.check_collision(start_state, options={"_skip_set_robot_cell_state": True})
            except CollisionCheckInCollisionError as e:
                e.message = "The start_state is in collision." + e.message
                raise e

        # Options for Inverse Kinematics
        options["high_accuracy"] = options.get("high_accuracy", True)
        options["high_accuracy_threshold"] = options.get("high_accuracy_threshold", 1e-4)
        options["high_accuracy_max_iter"] = options.get("high_accuracy_threshold", 20)
        options["max_results"] = options.get("max_results", 1)
        # max_results = 1 removes the random search of the IK Engine

        # Iterate over the waypoints as segments
        intermediate_state = start_state.copy()  # type: RobotCellState
        joint_names = start_state.robot_configuration.joint_names
        start_configuration = start_state.robot_configuration
        trajectory = JointTrajectory(joint_names=joint_names, start_configuration=start_configuration)

        # Add the start configuration as the first point
        trajectory.points.append(JointTrajectoryPoint(list(start_configuration.joint_values)))
        for i in range(len(waypoints.target_frames) - 1):
            start_frame = waypoints.target_frames[i]
            end_frame = waypoints.target_frames[i + 1]

            frame_interpolator = FrameInterpolator(start_frame, end_frame, options)
            # First round of interpolation is performed with respect to the max_step parameter
            number_of_points = frame_interpolator.num_points
            current_step = 1
            while current_step <= number_of_points:
                # Prepare the input for IK
                frame = frame_interpolator.get_frame(current_step)
                # get_frame() expects `current_step` to be a whole number for equal spacing
                target = FrameTarget(frame)
                intermediate_state.robot_configuration.joint_values = trajectory.points[-1].joint_values
                # Note that the PyBullet IK also performs collision checking
                # Note that the PyBullet IK is gradient based and will snap to the nearest configuration of the start state
                # Note that we are running this IK solver without random search to ensure determinism
                # Note that planner.inverse_kinematics() will raise an InverseKinematicsError if no solution is found
                joint_positions, joint_names_sorted = planner.inverse_kinematics(
                    target, start_configuration, group, options
                )
                trajectory.points.append(JointTrajectoryPoint(joint_positions))
                print(
                    "Segment {} of {}, Point {} of {} = {}".format(
                        i + 1, len(waypoints.target_frames) - 1, current_step, number_of_points, joint_positions
                    )
                )
                current_step += 1


class FrameInterpolator(object):
    """A class to interpolate between two frames.

    Linear Cartesian interpolation is used for position
    and spherical interpolation is used for orientation.

    This object allows for custom control of interpolation spacing between two frames
    """

    def __init__(self, start_frame, end_frame, options):
        # type: (Frame, Frame, Dict) -> None
        self.start_frame = start_frame
        self.end_frame = end_frame
        self.options = options

        # Compute the Cartesian distance between the two frames
        # This is used to determine the number of interpolated points
        # with the max_step parameter

        distance = start_frame.point.distance_to_point(end_frame.point)
        # Floor + 1 to ensure at least one point
        self.num_points = int(distance / options["max_step"]) + 1

    def get_frame(self, step):
        # type: (float) -> Frame
        """Interpolates the waypoints and returns a joint trajectory."""
        raise NotImplementedError("interpolator() is not implemented yet.")
