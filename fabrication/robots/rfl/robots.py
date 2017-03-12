from __future__ import print_function


class Configuration(object):
    """Represents a configuration of an RFL robot based on its
    coordinates (position of the gantry system) and joint angle values.

    Args:
        coordinates (:obj:`list` of :obj:`float`): Gantry position
            in x, y, z in meters.
        joint_values (:obj:`list` of :obj:`float`): 6 joint values
            expressed in degrees.
    """
    def __init__(self, coordinates, joint_values):
        if len(coordinates) != 3:
            raise ValueError('Expected 3 floats: x, y, z but got %d' % len(coordinates))
        if len(joint_values) != 6:
            raise ValueError('Expected 6 floats expressed in degrees, but got %d' % len(joint_values))

        self.coordinates = coordinates
        self.joint_values = joint_values

    def __str__(self):
        return "xyz: %s, joints: %s" % (self.coordinates, self.joint_values)


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

    def set_config(self, config):
        """Moves the robot the the specified configuration.

        Args:
            config (:class:`.Configuration`): Instance of robot's
                configuration.

        Examples:

            >>> from compas_fabrication.fabrication.robots.rfl import Simulator
            >>> with Simulator() as simulator:
            ...     robot = Robot('A', simulator)
            ...     robot.set_config(Configuration(
            ...                      [7.6, -4.5, -4.5],
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
