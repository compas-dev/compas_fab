from __future__ import print_function
import math

from ..robot import BaseConfiguration
from ..robot import Robot as BaseRobot


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


class Robot(BaseRobot):
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
    SUPPORTED_ROBOTS = (11, 12, 21, 22)
    ROBOT_SETTINGS = {
        11: {'name': 'A', 'base_external_axes': [7000, -2000, -4000]},
        12: {'name': 'B', 'base_external_axes': [7000, -10000, -4000]},
        21: {'name': 'C', 'base_external_axes': [30000, -2000, -4000]},
        22: {'name': 'D', 'base_external_axes': [30000, -10000, -4000]},
    }
    BASE_JOINT_VALUES = [0.] * 6

    def __init__(self, id, client=None):
        if id not in self.SUPPORTED_ROBOTS:
            raise ValueError('Robot ID is not valid, must be one of: ' + str(self.SUPPORTED_ROBOTS))
        self.id = id
        self.client = client
        self.name = self.ROBOT_SETTINGS[id]['name']
        self.index = self.SUPPORTED_ROBOTS.index(id)
        self.dof = 9
        self.external_axes = 3
        self.config_cls = Configuration

    def set_config(self, config):
        """Moves the robot the the specified configuration.

        Args:
            config (:class:`.Configuration`): Instance of robot's configuration.

        Examples:

            >>> from compas_fab.fab.robots.rfl import Simulator
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
                        self.ROBOT_SETTINGS[self.id]['base_external_axes']))
