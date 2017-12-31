from __future__ import print_function

from ..robot import BaseConfiguration
from ..robot import Robot as BaseRobot


class Configuration(BaseConfiguration):
    """Represents the configuration of an IRB robot based on its
    joint angle values and external axes values (if any).
    """

    @classmethod
    def from_joints_and_external_axes(cls, joint_values, external_axes):
        """Construct a configuration from a list of joint values and external
        axes values.

        Args:
            joint_values (:obj:`list` of :obj:`float`): 6 joint values
                expressed in degrees.
            external_axes (:obj:`list` of :obj:`float`): Linear axis position
                in x in millimeters.
        """
        if len(joint_values) != 6:
            raise ValueError('Expected 6 floats expressed in degrees, but got %d' % len(joint_values))
        if external_axes and len(external_axes) != 1:
            raise ValueError('Expected 1 floats: x but got %d' % len(external_axes))

        return cls.from_data({'joint_values': joint_values, 'external_axes': external_axes})


class IRB4600(BaseRobot):
    """Represents an instance of the IRB4600 ABB robots, optionally on a linear axis.

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
    SUPPORTED_ROBOTS = [1]
    ROBOT_SETTINGS = {
        1: {'name': 'A'},
    }
    BASE_JOINT_VALUES = [0.] * 6

    def __init__(self, id, client=None):
        if id not in self.SUPPORTED_ROBOTS:
            raise ValueError('Robot ID is not valid, must be one of: ' + str(self.SUPPORTED_ROBOTS))
        self.id = id
        self.client = client
        # TODO: Get rid of robot settings and supported ROBOT_SETTINGS
        # index is only needed for internal V-REP usage, so it should be related
        # to Simulator class, not specific robot classes
        self.name = self.ROBOT_SETTINGS[id]['name']
        self.index = self.SUPPORTED_ROBOTS.index(id)
        self.dof = 6
        # TODO: For the time being, the external linear axis is not supported
        self.external_axes = 0
        self.config_cls = Configuration

    def set_config(self, config):
        """Moves the robot the the specified configuration.

        Args:
            config (:class:`.Configuration`): Instance of robot's configuration.

        Examples:

            >>> from compas_fab.fab.robots import Simulator
            >>> with Simulator() as simulator:
            ...     robot = IRB4600(1, simulator)
            ...     robot.set_config(Configuration.from_joints(
            ...                      [90, 0, 0, 0, 0, -90]))
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
        self.set_config(Configuration.from_joints(self.BASE_JOINT_VALUES))
