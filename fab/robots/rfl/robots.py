from __future__ import print_function
import math

from ..robot import BaseConfiguration
from ..robot import Robot as BaseRobot


class Configuration(BaseConfiguration):
    """Represents the configuration of an RFL robot based on its
    joint angle values and external axes values in the gantry system.
    """

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
        client (:obj:`object`): A client to execute the commands
            such as :class:`..Simulator`.
        client_options (:obj:`dict`): Dictionary containing client-specific options.

    Attributes:
        client (:obj:`object`): A client to execute the commands
            such as :class:`..Simulator`.
        client_options (:obj:`dict`): Dictionary of client-specific options.
        dof (:obj:`int`): Degrees of freedom.
        external_axes (:obj:`int`): Number of external axes.
    """
    SUPPORTED_ROBOTS = (11, 12, 21, 22)
    ROBOT_SETTINGS = {
        11: {'name': 'A', 'index': 0, 'base_external_axes': [7000, -2000, -4000], 'base_joint_values': [0.] * 6},
        12: {'name': 'B', 'index': 1, 'base_external_axes': [7000, -10000, -4000], 'base_joint_values': [0.] * 6},
        21: {'name': 'C', 'index': 2, 'base_external_axes': [30000, -2000, -4000], 'base_joint_values': [0.] * 6},
        22: {'name': 'D', 'index': 3, 'base_external_axes': [30000, -10000, -4000], 'base_joint_values': [0.] * 6},
    }
    SIMULATION_OPTIONS = {'simulation_script': 'RFL'}

    def __init__(self, client=None, client_options=None):
        super(Robot, self).__init__(client, client_options)

        if client_options['id'] not in self.SUPPORTED_ROBOTS:
            raise ValueError('Robot ID is not valid, must be one of: ' + str(self.SUPPORTED_ROBOTS))

        self.client = client
        self.dof = 9
        self.external_axes = 3
        self.config_cls = Configuration

    @classmethod
    def get_options(cls, identifier):
        """"Get client options for a specific robot."""
        if identifier not in Robot.SUPPORTED_ROBOTS:
            raise ValueError(
                'Robot identifier is not valid, must be one of: ' + str(Robot.SUPPORTED_ROBOTS))

        return dict([('id', identifier)] + Robot.ROBOT_SETTINGS[identifier].items() + Robot.SIMULATION_OPTIONS.items())

    def reset_config(self):
        """Resets a robot's configuration to a safe initial position."""
        self.set_config(Configuration.from_joints_and_external_axes(
            self.client_options['base_joint_values'],
            self.client_options['base_external_axes']))
