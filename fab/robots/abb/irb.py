from __future__ import print_function

from compas_fab.fab.robots import BaseConfiguration
from compas_fab.fab.robots import Robot as BaseRobot


class Configuration(BaseConfiguration):
    """Represents the configuration of an IRB robot based on its
    joint angle values and external axes values (if any).
    """
    pass

class IRB4600(BaseRobot):
    """Represents an instance of the IRB4600 ABB robots, optionally on a linear axis.

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
    def __init__(self, client=None, client_options=None):
        super(IRB4600, self).__init__(client, client_options)

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
            ...     robot = IRB4600(simulator, {'id': 1, 'name': 'A', 'index': 0, 'simulation_script': 'LinearAxis'})
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
        self.set_config(Configuration.from_joints([0.] * self.dof))
