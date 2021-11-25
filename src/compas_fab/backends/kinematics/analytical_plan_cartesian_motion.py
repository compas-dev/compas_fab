from compas_fab.backends.interfaces import PlanCartesianMotion
from compas_fab.robots import JointTrajectory
from compas_fab.robots import JointTrajectoryPoint
from compas_fab.backends.kinematics.utils import find_closest_configuration

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

        # convert the frame WCF to RCF
        base_frame = robot.get_base_frame(group=group, full_configuration=start_configuration)
        frames_RCF = [base_frame.to_local_coordinates(frame_WCF) for frame_WCF in frames_WCF]

        # how many to divide in between
        # stepsize??
        
        configurations_along_path = []
        for frame in frames_RCF:
            configurations = list(robot.iter_inverse_kinematics(frame, options=options))
            configurations_along_path.append(configurations)

        paths = []
        for configurations in zip(*configurations_along_path):
            if all(configurations):
                paths.append(configurations)
        
        print("len(paths", len(paths))

        # now make smooth

        # Select the one that is closest to the start frame?
        idx = find_closest_configuration([path[0] for path in paths], start_configuration)
        selected_path = paths[idx]

        trajectory = JointTrajectory()
        trajectory.fraction = len(selected_path[0])/len(frames_RCF)
        trajectory.joint_names = selected_path[0].joint_names  
        trajectory.points = [JointTrajectoryPoint(config.joint_values, config.joint_types) for config in selected_path]
        trajectory.start_configuration = robot.merge_group_with_full_configuration(selected_path[0], start_configuration, group)

        return trajectory