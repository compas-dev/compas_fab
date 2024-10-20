import math

from compas.geometry import argmin

from compas_fab.backends.interfaces import PlanCartesianMotion
from compas_fab.backends.kinematics.exceptions import CartesianMotionError
from compas_fab.backends.kinematics.utils import smallest_joint_angles
from compas_fab.robots import JointTrajectory
from compas_fab.robots import JointTrajectoryPoint
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import PointAxisWaypoints

from compas import IPY

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from compas_fab.backends import AnalyticalPyBulletPlanner  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import Waypoints  # noqa: F401

        from typing import Optional  # noqa: F401
        from typing import Dict  # noqa: F401


class AnalyticalPlanCartesianMotion(PlanCartesianMotion):
    """ """

    def plan_cartesian_motion(self, waypoints, start_state, group=None, options=None):
        # type: (Waypoints, RobotCellState, Optional[str], Optional[Dict]) -> JointTrajectory
        """Calculates a cartesian motion path (linear in tool space).

        Parameters
        ----------
        waypoints : :class:`compas_fab.robots.Waypoints`
            The waypoints for the robot to follow.
        start_state : :class:`compas_fab.robots.RobotCellState`
            The starting state of the robot cell at the beginning of the motion.
            The attribute `robot_configuration`, must be provided.
        group : str, optional
            The planning group used for calculation.
        options : dict, optional
            Dictionary containing the key-value pairs that are passed to :func:`compas_fab.robots.Robot.iter_inverse_kinematics`

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
            The calculated trajectory.

        Notes
        -----
        This will only work with robots that have 6 revolute joints.
        """

        if isinstance(waypoints, FrameWaypoints):
            return self._plan_cartesian_motion_with_frame_waypoints(waypoints, start_state, group, options)
        elif isinstance(waypoints, PointAxisWaypoints):
            return self._plan_cartesian_motion_with_point_axis_waypoints(waypoints, start_state, group, options)
        else:
            raise TypeError("Unsupported waypoints type {}".format(type(waypoints)))

    def _plan_cartesian_motion_with_frame_waypoints(self, waypoints, start_state, group=None, options=None):
        # type: (Waypoints, RobotCellState, Optional[str], Optional[Dict]) -> JointTrajectory
        """Calculates a cartesian motion path with frame waypoints.

        Planner behavior:
        - If multiple paths are possible (i.e. due to multiple IK results), only the one that is closest to the start_configuration is returned.
        - The path is checked to ensure that the joint values are continuous and that revolution values are the smallest possible.
        - There is no interpolation in between frames (i.e. 'max_step' parameter is not supported), only the input frames are used.
        """
        planner = self  # type: AnalyticalPyBulletPlanner
        robot_cell = planner.client.robot_cell

        waypoints = waypoints.normalized_to_meters()

        # TODO: This function needs some serious rewrites as the continuity check is a bit flawed.

        # TODO: Convert the target frames to the robot's base frame using new method in planner class
        target_frames = waypoints.target_frames

        # convert the frame WCF to RCF
        pcf_frames = robot_cell.target_frames_to_pcf(start_state, target_frames, waypoints.target_mode, group)

        # 'keep_order' is set to True, so that iter_inverse_kinematics will return the configurations in the same order across all frames
        options = options or {}
        options.update({"keep_order": True})

        # iterate over all input frames and calculate the inverse kinematics, no interpolation in between frames
        configurations_along_path = []
        # TODO: Change to planner.iter_inverse_kinematics, should be able to list out all possible
        for frame in pcf_frames:
            configurations = list(planner.iter_inverse_kinematics(frame, options=options))
            configurations_along_path.append(configurations)

        # Analytical backend only supports robots with finite IK solutions
        # For 6R articulated robots, there is a maximum of 8 possible paths, corresponding to the 8 possible IK solutions for each frame
        # The `options.update({"keep_order": True})` ensures that the order of the configurations is the same across all frames
        # but this also cause some configurations to be None, if no solution was found.

        # The `all(configurations)` below is used to check if all configurations in a path are present.
        # indicating that a complete trajectory was found.
        paths = []
        for configurations in zip(*configurations_along_path):
            if all(configurations):
                paths.append(configurations)

        if not len(paths):
            raise CartesianMotionError("No complete trajectory found.")

        # now select the path that is closest to the start configuration.
        first_configurations = [path[0] for path in paths]
        diffs = [
            sum([abs(d) for d in start_state.robot_configuration.iter_differences(c)]) for c in first_configurations
        ]
        idx = argmin(diffs)

        path = paths[idx]
        path = self.smooth_configurations(path)
        trajectory = JointTrajectory()
        trajectory.fraction = len(path) / len(pcf_frames)
        # Technically trajectory.fraction should always be 1.0 because otherwise, the path would be rejected earlier
        trajectory.joint_names = path[0].joint_names
        trajectory.points = [JointTrajectoryPoint(config.joint_values, config.joint_types) for config in path]
        first_configuration = robot_cell.fill_configuration_with_joint_names(path[0])
        trajectory.start_configuration = robot_cell.zero_full_configuration().merged(first_configuration)
        return trajectory

    def _plan_cartesian_motion_with_point_axis_waypoints(self, waypoints, start_state, group=None, options=None):
        # type: (Waypoints, RobotCellState, Optional[str], Optional[Dict]) -> JointTrajectory
        """Planning Cartesian motion with PointAxisWaypoints is not yet implemented in the Analytical backend."""
        raise NotImplementedError(
            "Planning Cartesian motion with PointAxisWaypoints is not yet implemented in the Analytical backend."
        )

    def smooth_configurations(self, configurations):
        joint_values_corrected = []
        prev = smallest_joint_angles(configurations[0].joint_values)
        joint_values_corrected.append(prev)

        for i in range(1, len(configurations)):
            curr = configurations[i].joint_values
            corrected = []
            for p, c in zip(prev, curr):
                c1 = c / abs(c) * (abs(c) % (2 * math.pi))
                c2 = c1 - 2 * math.pi
                c3 = c1 + 2 * math.pi
                values = [c1, c2, c3]
                diffs = [math.fabs(p - v) for v in values]
                idx = argmin(diffs)
                corrected.append(values[idx])
            prev = corrected
            joint_values_corrected.append(corrected)

        # now that the values are continuous, try to bring them all "down"
        for i, j in enumerate(zip(*joint_values_corrected)):
            minj, maxj = min(j), max(j)
            v1 = minj + maxj
            v2 = minj - 2 * math.pi + maxj - 2 * math.pi
            v3 = minj + 2 * math.pi + maxj + 2 * math.pi
            values = [math.fabs(v) for v in [v1, v2, v3]]
            idx = argmin(values)
            if idx == 1:
                for k in range(len(joint_values_corrected)):
                    joint_values_corrected[k][i] -= 2 * math.pi
            elif idx == 2:
                for k in range(len(joint_values_corrected)):
                    joint_values_corrected[k][i] += 2 * math.pi

        for i in range(len(joint_values_corrected)):
            configurations[i].joint_values = joint_values_corrected[i]
        return configurations
