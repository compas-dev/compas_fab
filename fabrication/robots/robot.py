from __future__ import print_function


class BaseConfiguration(object):
    """Represents the configuration of a robot based on its
    joint angle values and coordinates (position of external axis system, if any).

    Attributes:
        joint_values (:obj:`list` of :obj:`float`): Joint values expressed
            in degrees.
        coordinates (:obj:`list` of :obj:`float`): Position on the external axis
            system (if available).

    Examples:

        >>> from compas_fabrication.fabrication.robots import BaseConfiguration
        >>> config = BaseConfiguration.from_data({'joint_values': [90., 0., 0.]})
        >>> config.joint_values
        [90.0, 0.0, 0.0]


        >>> from compas_fabrication.fabrication.robots import BaseConfiguration
        >>> config = BaseConfiguration.from_data({'joint_values': [90., 0., 0., 0., 180., 45.],\
                                                 'coordinates': [8312.0]})
        >>> str(config)
        'joints: [90.0, 0.0, 0.0, 0.0, 180.0, 45.0], coordinates: [8312.0]'

    """
    def __init__(self):
        self.joint_values = None
        self.coordinates = None

    def __str__(self):
        return "joints: %s, coordinates: %s" % (self.joint_values, self.coordinates)

    @classmethod
    def from_joints(cls, joint_values):
        """Construct a configuration from joint values and coordinates.

        Args:
            joint_values (:obj:`list` of :obj:`float`): Joint values expressed
                in degrees.
            coordinates (:obj:`list` of :obj:`float`): Position on the external axis
                system (if available).

        Returns:
            Configuration: A :class:`.Configuration` instance.
        """
        return cls.from_joints_and_coordinates(joint_values, None)

    @classmethod
    def from_joints_and_coordinates(cls, joint_values, coordinates=None):
        """Construct a configuration from joint values and coordinates.

        Args:
            joint_values (:obj:`list` of :obj:`float`): Joint values expressed
                in degrees.
            coordinates (:obj:`list` of :obj:`float`): Position on the external axis
                system (if available).

        Returns:
            Configuration: A :class:`.Configuration` instance.
        """
        return cls.from_data({'joint_values': joint_values, 'coordinates': coordinates})

    @classmethod
    def from_data(cls, data):
        """Construct a configuration from its data representation.

        Args:
            data (`dict`): The data dictionary.

        Returns:
            Configuration: A :class:`.Configuration` instance.
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
            'coordinates': self.coordinates
        }

    @data.setter
    def data(self, data):
        self.joint_values = data.get('joint_values') or None
        self.coordinates = data.get('coordinates') or None


class Pose(object):
    """Represents a robot pose described as a 4x4 transformation matrix.

    Attributes:
        values (:obj:`list` of :obj:`float`): list of 12 or 16 values representing a 4x4 matrix.
            If 12 values are provided, the last row is assumed to be ``[0 0 0 1]``.
    """
    def __init__(self):
        self.values = []

    def __str__(self):
        return "[%s, %s, %s, %s]" % (self.values[0:4], self.values[4:8], self.values[8:12], self.values[12:16])

    @classmethod
    def from_list(cls, list):
        """Construct a pose from a list of 12 or 16 :obj:`float` values.

        Args:
            list (:obj:`list` of :obj:`float`): list of 12 or 16 values representing a 4x4 matrix.

        Returns:
            Pose: A :class:`.Pose` instance.
        """
        return cls.from_data({'values': list})

    @classmethod
    def from_data(cls, data):
        """Construct a pose from its data representation.

        Args:
            data (`dict`): The data dictionary.

        Returns:
            Pose: A :class:`.Pose` instance.
        """
        pose = cls()
        pose.data = data
        return pose

    def to_data(self):
        """Return the data dict that represents the pose, and from which it can
        be reconstructed."""
        return self.data

    @property
    def data(self):
        """:obj:`dict` : The data representing the pose."""
        return {'values': self.values}

    @data.setter
    def data(self, data):
        values = data.get('values') or None

        if len(values) == 12:
            values.extend([0., 0., 0., 1.])
        elif len(values) != 16:
            raise ValueError('Expected 12 or 16 floats but got %d' % len(values))

        self.values = values
