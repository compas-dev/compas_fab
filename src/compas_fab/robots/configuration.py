
class Configuration(object):
    """Represents the configuration of a robot based on its
    joint angle values and external axes values (if any).

    Attributes:
        joint_values (:obj:`list` of :obj:`float`): Joint values expressed
            in degrees.
        external_axes (:obj:`list` of :obj:`float`): Position on the external axis
            system expressed in millimeters (if available).

    Examples:

        >>> from compas_fab.robots import Configuration
        >>> config = Configuration.from_data({'joint_values': [90., 0., 0.]})
        >>> config.joint_values
        [90.0, 0.0, 0.0]


        >>> from compas_fab.robots import Configuration
        >>> config = Configuration.from_data({'joint_values': [90., 0., 0., 0., 180., 45.],\
                                              'external_axes': [8312.0]})
        >>> str(config)
        'joints: [90.0, 0.0, 0.0, 0.0, 180.0, 45.0], external_axes: [8312.0]'

    """

    def __init__(self):
        self.joint_values = None
        self.external_axes = None

    def __str__(self):
        return "joints: %s, external_axes: %s" % (self.joint_values, self.external_axes)

    @classmethod
    def from_joints(cls, joint_values):
        """Construct a configuration from joint values.

        Args:
            joint_values (:obj:`list` of :obj:`float`): Joint values expressed
                in degrees.

        Returns:
            Configuration: A :class:`.Configuration` instance.
        """
        return cls.from_joints_and_external_axes(joint_values, None)

    @classmethod
    def from_joints_and_external_axes(cls, joint_values, external_axes=None):
        """Construct a configuration from joint values and external axes values.

        Args:
            joint_values (:obj:`list` of :obj:`float`): Joint values expressed
                in degrees.
            external_axes (:obj:`list` of :obj:`float`): Position on the external axis
                system in millimeters (if available).

        Returns:
            Configuration: A :class:`.Configuration` instance.
        """
        return cls.from_data({'joint_values': joint_values, 'external_axes': external_axes})

    @classmethod
    def from_data(cls, data):
        """Construct a configuration from its data representation.

        Args:
            data (`dict`): The data dictionary.

        Returns:
            Configuration: A :class:`Configuration` instance.
        """
        config = cls()
        config.data = data
        return config

    def to_data(self):
        """Return the data dict that represents the configuration, and from which it can
        be reconstructed."""
        return self.data

    @property
    def data(self):
        """:obj:`dict` : The data representing the configuration.

        By assigning a data dict to this property, the current data of the configuration
        will be replaced by the data in the dict. The data getter and setter should
        always be used in combination with each other.
        """
        return {
            'joint_values': self.joint_values,
            'external_axes': self.external_axes
        }

    @data.setter
    def data(self, data):
        self.joint_values = data.get('joint_values') or None
        self.external_axes = data.get('external_axes') or None


if __name__ == "__main__":
    q = [4.5, 1.7, 0.5, 2.1, 0.1, 2.1]
    configuration = Configuration.from_joints(q)
    print(configuration)
    from compas_fab.robots import Configuration
    from compas_fab.robots import Pose
