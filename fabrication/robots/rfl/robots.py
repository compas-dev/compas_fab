from __future__ import print_function


# TODO: This should inherit from compas_fabrication.fabrication.robots.Robot
# once that is in place.
class Robot(object):
    """Represents an instance of the ABB robots of the Robotic Fabrication Lab.

    Communication to the robot is delegated to the `client` instance
    passed when initializing the robot.

    Args:
        name (:obj:`str`): Robot identifier.
        client (:obj:`object`): A client to execute the commands
            such as :class:`.Simulator`.
        index (:obj:`int`): Robot index (for internal use).
    """
    SUPPORTED_ROBOTS = ('A', 'B', 'C', 'D')

    def __init__(self, name, client=None):
        if name not in self.SUPPORTED_ROBOTS:
            raise ValueError('Robot name is not valid')
        self.name = name
        self.client = client
        self.index = self.SUPPORTED_ROBOTS.index(name)

    def set_config(self, gantry_values, joint_values):
        """Moves the robot the the specified configuration.

        Args:
            gantry_values (:obj:`list` of :obj:`float`): Gantry position
                in x, y, z in meters.
            joint_values (:obj:`list` of :obj:`float`): 6 joint values
                expressed in degrees.

        Examples:

            >>> from compas_fabrication.fabrication.robots.rfl import Simulator
            >>> with Simulator() as simulator:
            ...     robot = Robot('A', simulator)
            ...     robot.set_config(
            ...                      [7.6, -4.5, -4.5],
            ...                      [90, 0, 0, 0, 0, -90])
            ...

        """
        self.client.set_robot_config(self, gantry_values, joint_values)
