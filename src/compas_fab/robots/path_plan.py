from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.robots.configuration import Configuration
from compas_fab.robots.robot import Robot

__all__ = [
    'PathPlan',
]


class PathPlan(object):
    """Describes a complete path plan for one or more robots.

    Attributes
    ----------
    paths : :obj:`dict`
        Dictionary keyed by the robot identifier where the values
        are instances of :class:`Configuration`. Robots that do not move during the
        plan only have one configuration in their values.
    """

    def __init__(self):
        self.paths = {}

    def add_robot_plan(self, robot, path_plan):
        """Adds a path plan for a specific robot.

        Parameters
        ----------
        robot : :class:`Robot`
            Instance of robot.
        path_plan : :obj:`list` of :class:`Configuration`
            List of configurations representing a full path.
        """
        self.paths[robot.name] = path_plan

    def get_robot_plan(self, robot):
        """Gets the path plan for a specific robot.

        Parameters
        ----------
        robot : :class:`Robot`
            Instance of robot.

        Returns
        -------
        list
            List of configurations representing a full path.
        """
        if robot.name not in self.paths:
            raise ValueError('No path plan stored for the specified robot: ' + robot.name)

        return self.paths[robot.name]

    def all_paths(self):
        """Iterator over all paths currently defined."""
        for key in self.paths:
            yield key, self.paths[key]
