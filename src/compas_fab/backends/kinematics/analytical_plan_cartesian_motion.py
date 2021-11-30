import math
from compas.geometry import argmin
from compas_fab.robots import JointTrajectory
from compas_fab.robots import JointTrajectoryPoint
from compas_fab.backends.interfaces import PlanCartesianMotion
from compas_fab.backends.kinematics.utils import smallest_joint_angles


class AnalyticalPlanCartesianMotion(PlanCartesianMotion):
    """
    """

    def __init__(self, client=None):
        self.client = client

    def plan_cartesian_motion(self, robot, frames_WCF, start_configuration=None, group=None, options=None):
        """Calculates a cartesian motion path (linear in tool space).

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which the cartesian motion path is being calculated.
        frames_WCF: list of :class:`compas.geometry.Frame`
            The frames through which the path is defined.
        start_configuration: :class:`Configuration`, optional
            The robot's full configuration, i.e. values for all configurable
            joints of the entire robot, at the starting position.
        group: str, optional
            The planning group used for calculation.
        options: dict, optional
            Dictionary containing kwargs for arguments specific to
            the client being queried.

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
            The calculated trajectory.
        """
        # what is the expected behaviour of that function?
        # - Return all possible paths or select only the one that is closest to the start_configuration?
        # - Do we use a stepsize to sample in between frames or use only the input frames?

        # convert the frame WCF to RCF
        base_frame = robot.get_base_frame(group=group, full_configuration=start_configuration)
        frames_RCF = [base_frame.to_local_coordinates(frame_WCF) for frame_WCF in frames_WCF]

        options = options or {}
        options.update({"keep_order": True})

        configurations_along_path = []
        for frame in frames_RCF:
            configurations = list(robot.iter_inverse_kinematics(frame, options=options))
            configurations_along_path.append(configurations)

        # TODO: how to handle path not complete?

        paths = []
        for configurations in zip(*configurations_along_path):
            if all(configurations):
                paths.append(configurations)

        # now select the path that is closest to the start configuration.
        first_configurations = [path[0] for path in paths]
        diffs = [sum([abs(d) for d in start_configuration.iter_differences(c)]) for c in first_configurations]
        idx = argmin(diffs)

        path = paths[idx]
        path = self.smooth_configurations(path)
        trajectory = JointTrajectory()
        trajectory.fraction = len(path)/len(frames_RCF)
        trajectory.joint_names = path[0].joint_names
        trajectory.points = [JointTrajectoryPoint(config.joint_values, config.joint_types) for config in path]
        trajectory.start_configuration = robot.merge_group_with_full_configuration(path[0], start_configuration, group)
        return trajectory

    def smooth_configurations(self, configurations):
        joint_values_corrected = []
        prev = smallest_joint_angles(configurations[0].joint_values)
        joint_values_corrected.append(prev)

        for i in range(1, len(configurations)):
            curr = configurations[i].joint_values
            corrected = []
            for p, c in zip(prev, curr):
                c1 = c/abs(c) * (abs(c) % (2 * math.pi))
                c2 = c1 - 2 * math.pi
                c3 = c1 + 2 * math.pi
                values = [c1, c2, c3]
                diffs = [math.fabs(p - c) for c in values]
                idx = diffs.index(min(diffs))
                corrected.append(values[idx])
            prev = corrected
            joint_values_corrected.append(corrected)

        # now that the values are continuous, try to bring them all "down"
        for i, j in enumerate(zip(*joint_values_corrected)):
            v1 = min(j) + max(j)
            v2 = min(j) - 2 * math.pi + max(j) - 2 * math.pi
            v3 = min(j) + 2 * math.pi + max(j) + 2 * math.pi
            values = [math.fabs(v) for v in [v1, v2, v3]]
            idx = values.index(min(values))
            if idx == 1:
                for k in range(len(joint_values_corrected)):
                    joint_values_corrected[k][i] -= 2 * math.pi
            elif idx == 2:
                for k in range(len(joint_values_corrected)):
                    joint_values_corrected[k][i] -= 2 * math.pi

        for i in range(len(joint_values_corrected)):
            configurations[i].joint_values = joint_values_corrected[i]
        return configurations
