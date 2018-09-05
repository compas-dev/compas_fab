from __future__ import print_function
import math

from ..configuration import Configuration as BaseConfiguration


class Configuration(BaseConfiguration):
    """Represents the configuration of an RFL robot based on its
    joint angle values and external axes values in the gantry system.
    """

    @classmethod
    def from_joints_and_external_axes(cls, joint_values, external_axes):
        """Construct a configuration from a list of joint values and external
        axes values.

        Args:
            joint_values (:obj:`list` of :obj:`float`): 6 joint values
                expressed in degrees.
            external_axes (:obj:`list` of :obj:`float`): Gantry position
                in x, y, z in millimeters.
        """
        if len(joint_values) != 6:
            raise ValueError('Expected 6 floats expressed in degrees, but got %d' % len(joint_values))
        if len(external_axes) != 3:
            raise ValueError('Expected 3 floats: x, y, z but got %d' % len(external_axes))

        return cls.from_data({'joint_values': joint_values, 'external_axes': external_axes})

    @classmethod
    def from_radians_list(cls, list_of_floats):
        """Construct a configuration from a flat list of 6 joint values expressed in radians
        and 3 axis values in millimeters.

        Args:
            list_of_floats (:obj:`list` of :obj:`float`): 9 joint values where the first 6 are radians
                of the joint values, and the last 3 are gantry positions in millimeters.
        """
        angles = map(math.degrees, list_of_floats[0:6])
        return cls.from_joints_and_external_axes(angles, list_of_floats[6:])

    @classmethod
    def from_degrees_list(cls, list_of_floats):
        """Construct a configuration from a flat list of 6 joint values expressed in degrees
        and 3 axis values in millimeters.

        Args:
            list_of_floats (:obj:`list` of :obj:`float`): 9 joint values where the first 6 are degrees
                of the joint values, and the last 3 are gantry positions in millimeters.
        """
        return cls.from_joints_and_external_axes(list_of_floats[0:6], list_of_floats[6:])


class PathPlan(object):
    """Represents a complete path planning for one or more robots.

    Attributes:
        paths (:obj:`dict`): Dictionary keyed by the robot identifier where the values
            are instances of :class:`.Configuration`. Robots that do not move during the
            plan only have one configuration in their values.
    """
    def __init__(self):
        self.paths = {}

    def add_robot_plan(self, robot, path_plan):
        """Adds a path plan for a specific robot.

        Args:
            robot (:class:`.Robot`): Instance of robot.
            path_plan (:obj:`list``or :class:`.`Configuration`): List of configurations
                representing a full path.
        """
        self.paths[robot.name] = path_plan

    def get_robot_plan(self, robot):
        """Gets the path plan for a specific robot.

        Args:
            robot (:class:`.Robot`): Instance of robot.

        Returns:
            List of configurations representing a full path.
        """
        if robot.name not in self.paths:
            raise ValueError('No path plan stored for the specified robot: ' + robot.name)

        return self.paths[robot.name]

    def all_paths(self):
        """Iterator over all paths currently defined."""
        for key in self.paths:
            yield key, self.paths[key]


# TODO: This should inherit from compas_fab.robots.Robot
# once that is in place.
class Robot(object):
    """Represents an instance of the ABB robots of the Robotic Fabrication Lab.

    Communication to the robot is delegated to the `client` instance
    passed when initializing the robot.

    Args:
        id (:obj:`int`): Robot identifier.
        client (:obj:`object`): A client to execute the commands
            such as :class:`.Simulator`.

    Attributes:
        id (:obj:`int`): Robot identifier.
        client (:obj:`object`): A client to execute the commands
            such as :class:`.Simulator`.
        index (:obj:`int`): Robot index (for internal use).
        dof (:obj:`int`): Degrees of freedom.
    """
    SUPPORTED_ROBOTS = ['A', 'B', 'C', 'D']
    ROBOT_SETTINGS = {
        'A': {'id': 11, 'base_external_axes': [7000, -2000, -4000]},
        'B': {'id': 12, 'base_external_axes': [7000, -10000, -4000]},
        'C': {'id': 21, 'base_external_axes': [30000, -2000, -4000]},
        'D': {'id': 22, 'base_external_axes': [30000, -10000, -4000]},
    }
    BASE_JOINT_VALUES = [0.] * 6

    def __init__(self, name, client=None):
        if name not in self.SUPPORTED_ROBOTS:
            raise ValueError('Robot name is not valid, must be one of: ' + str(self.SUPPORTED_ROBOTS))
        self.name = name
        self.client = client
        self.id = self.ROBOT_SETTINGS[name]['id']
        self.index = self.SUPPORTED_ROBOTS.index(name)
        self.dof = 9

    def set_config(self, config):
        """Moves the robot the the specified configuration.

        Args:
            config (:class:`.Configuration`): Instance of robot's configuration.

        Examples:

            >>> from compas_fab.robots.rfl import Simulator
            >>> with Simulator() as simulator:
            ...     robot = Robot(11, simulator)
            ...     robot.set_config(Configuration.from_joints_and_external_axes(
            ...                      [90, 0, 0, 0, 0, -90],
            ...                      [7600, -4500, -4500]))
            ...

        """
        self.client.set_robot_config(self, config)

    def get_config(self):
        """Gets the current configuration of the robot.

        Returns:
            config: Instance of (:class:`.Configuration`).
        """
        return self.client.get_robot_config(self)

    def reset_config(self):
        """Resets a robot's configuration to a safe initial position."""
        self.set_config(Configuration.from_joints_and_external_axes(
                        self.BASE_JOINT_VALUES,
                        self.ROBOT_SETTINGS[self.name]['base_external_axes']))
