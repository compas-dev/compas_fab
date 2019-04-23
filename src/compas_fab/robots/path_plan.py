from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


__all__ = [
    'PathPlan',
]


class PathPlan(object):
    """Describes a complete path plan for one or more robots.

    Attributes
    ----------
    trajectories : :obj:`dict`
        Dictionary keyed by the robot identifier where the values
        are instances of :class:`compas_fab.robots.Trajectory`.
        Robots that do not move during the plan have an empty
        trajectory.
    """

    def __init__(self):
        self.trajectories = {}

    def add_robot_trajectory(self, robot, trajectory):
        """Adds a trajectory for a specific robot.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            Instance of robot.
        trajectory : :class:`compas_fab.robots.Trajectory`
            Trajectory of the robot.
        """
        self.trajectories[robot.name] = trajectory

    def get_robot_trajectory(self, robot):
        """Gets the trajectory for a specific robot.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            Instance of robot.

        Returns
        -------
        :class:`compas_fab.robots.Trajectory`
            Trajectory of the robot.
        """
        if robot.name not in self.trajectories:
            raise ValueError('No trajectory stored for the specified robot: '
                             + robot.name)

        return self.trajectories[robot.name]

    def all_trajectories(self):
        """Iterator over all trajectories currently defined."""
        for key in self.trajectories:
            yield key, self.trajectories[key]
