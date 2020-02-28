from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.robots import Joint

__all__ = [
    'Configuration',
]


class Configuration(object):
    """Represents the configuration of a robot based on the state of its joints.
    This concept is also refered to as `Joint State`.

    Attributes
    ----------
    values : :obj:`list` of :obj:`float`
        Joint values expressed in radians or meters, depending on the respective type.
    types : list of :class:`compas.robots.Joint.TYPE`
        Joint types, e.g. a list of `compas.robots.Joint.REVOLUTE` for revolute joints.
    joint_names : :obj:`list` of :obj:`str`, optional
        Joint names list

    Examples
    --------
    >>> config = Configuration.from_revolute_values([math.pi/2, 0., 0.])
    >>> config.values
    [1.5707963267948966, 0.0, 0.0]

    >>> from compas_fab.robots import Configuration
    >>> config = Configuration.from_prismatic_and_revolute_values([8.312], [math.pi/2, 0., 0., 0., 2*math.pi, 0.8])
    >>> str(config)
    'Configuration((8.312, 1.571, 0.000, 0.000, 0.000, 6.283, 0.800), (2, 0, 0, 0, 0, 0, 0))'

    >>> from compas_fab.robots import Configuration
    >>> from compas.robots import Joint
    >>> config = Configuration([math.pi/2, 3., 0.1], [Joint.REVOLUTE, Joint.PRISMATIC, Joint.PLANAR])
    >>> str(config)
    'Configuration((1.571, 3.000, 0.100), (0, 2, 5))'

    """

    def __init__(self, values=None, types=None, joint_names=None):
        self._precision = '3f'
        self.values = list(values or [])
        self.types = list(types or [])
        self.joint_names = list(joint_names or [])

        if len(self.values) != len(self.types):
            raise ValueError("%d values must have %d types, but %d given." % (
                len(self.values), len(self.values), len(self.types)))

    def __str__(self):
        v_str = ('(' + ", ".join(['%.' + self._precision] * len(self.values)) + ')') % tuple(self.values)
        if len(self.joint_names):
            return "Configuration({}, {}, {})".format(v_str, tuple(self.types), tuple(self.joint_names))
        else:
            return "Configuration({}, {})".format(v_str, tuple(self.types))

    def __repr__(self):
        return self.__str__()

    @classmethod
    def from_revolute_values(cls, values):
        """Construct a configuration from revolute joint values in radians.

        Parameters
        ----------
        values : :obj:`list` of :obj:`float`
            Joint values expressed in radians.

        Returns
        -------
        :class:`Configuration`
             An instance of :class:`Configuration` instance.
        """
        values = list(values)
        return cls.from_data({'values': values, 'types': [Joint.REVOLUTE] * len(values)})

    @classmethod
    def from_prismatic_and_revolute_values(cls, prismatic_values, revolute_values):
        """Construct a configuration from prismatic and revolute joint values.

        Parameters
        ----------
        prismatic_values : :obj:`list` of :obj:`float`
            Positions on the external axis system in meters.
        revolute_values : :obj:`list` of :obj:`float`
            Joint values expressed in radians.

        Returns
        -------
        :class:`Configuration`
             An instance of :class:`Configuration` instance.
        """
        # Force iterables into lists
        prismatic_values = list(prismatic_values)
        revolute_values = list(revolute_values)
        values = prismatic_values + revolute_values
        types = [Joint.PRISMATIC] * \
            len(prismatic_values) + [Joint.REVOLUTE] * len(revolute_values)
        return cls.from_data({'values': values, 'types': types})

    @classmethod
    def from_data(cls, data):
        """Construct a configuration from its data representation.

        Parameters
        ----------
        data : :obj:`dict`
            The data dictionary.

        Returns
        -------
        :class:`Configuration`
             An instance of :class:`Configuration` instance.
        """
        config = cls()
        config.data = data
        return config

    def to_data(self):
        """Return the data dictionary that represents the configuration, and from
        which it can be reconstructed."""
        return self.data

    @property
    def data(self):
        """:obj:`dict` : The data representing the configuration.

        By assigning a data dictionary to this property, the current data of the
        configuration will be replaced by the data in the dict. The data getter
        and setter should always be used in combination with each other.
        """
        return {
            'values': self.values,
            'types': self.types,
            'joint_names': self.joint_names
        }

    @data.setter
    def data(self, data):
        self.values = data.get('values') or []
        self.types = data.get('types') or []
        self.joint_names = data.get('joint_names') or []

    @property
    def prismatic_values(self):
        """:obj:`list` of :obj:`float` : Prismatic joint values in meters.

        E.g. positions on the external axis system."""
        return [v for i, v in enumerate(self.values) if self.types[i] == Joint.PRISMATIC]

    @property
    def revolute_values(self):
        """:obj:`list` of :obj:`float` : Revolute joint values in radians."""
        return [v for i, v in enumerate(self.values) if self.types[i] == Joint.REVOLUTE]

    def copy(self):
        cls = type(self)
        return cls(self.values[:], self.types[:], self.joint_names[:])

    def scale(self, scale_factor):
        """Scales the joint positions of the current configuration.

        Only scalable joints are scaled, i.e. planar and prismatic joints.

        Parameters
        ----------
        scale_factor : float
            Scale factor

        Returns
        -------
        None
        """
        values_scaled = []

        for value, joint_type in zip(self.values, self.types):
            if joint_type in (Joint.PLANAR, Joint.PRISMATIC):
                value *= scale_factor
            values_scaled.append(value)

        self.values = values_scaled

    def scaled(self, scale_factor):
        """Returns a scaled copy of the joint positions of this configuration.

        Only scalable joints are scaled, i.e. planar and prismatic joints.

        Parameters
        ----------
        scale_factor : float
            Scale factor

        Returns
        -------
        None
        """
        config = self.copy()
        config.scale(scale_factor)
        return config
