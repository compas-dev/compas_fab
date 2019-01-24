from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import compas
from compas.robots import Joint

__all__ = [
    'Configuration',
]


class Configuration(object):
    """Represents the configuration of a robot based on the
    state of its joints. This concept is also refered to
    as `Joint State`.

    Attributes
    ----------
    values : :obj:`list` of :obj:`float`
        Joint values expressed in radians or meters, depending on the respective type.
    types : :obj:`list` of :class:`Joint.TYPE`
        Joint types, e.g. a list of ``Joint.REVOLUTE`` for revolute joints.

    Examples
    --------

    >>> from math import pi
    >>> from compas_fab.robots import Configuration
    >>> config = Configuration.from_revolute_values([pi/2, 0., 0.])
    >>> config.values
    [1.5707963267948966, 0.0, 0.0]

    >>> from math import pi
    >>> from compas_fab.robots import Configuration
    >>> config = Configuration.from_prismatic_and_revolute_values([8.312], [pi/2, 0., 0., 0., 2*pi, 0.8])
    >>> str(config)
    'Configuration(8.312, 1.571, 0.000, 0.000, 0.000, 6.283, 0.800)'

    >>> from math import pi
    >>> from compas_fab.robots import Configuration
    >>> from compas.robots import Joint
    >>> config = Configuration([pi/2, 3., 0.1], [Joint.REVOLUTE, Joint.PRISMATIC, Joint.PLANAR])
    >>> str(config)
    'Configuration(1.571, 3.000, 0.100)'

    """

    def __init__(self, values=[], types=[]):
        if len(values) != len(types):
            raise ValueError("%d values must have %d types, but %d given." % (len(values), len(values), len(types)))
        self.values = list(values)
        self.types = list(types)
        self._precision = '3f'

    def __str__(self):
        vs = '%.' + self._precision
        return "Configuration(%s, %s)" % ('(' + ", ".join([vs] * len(self.values)) % tuple(self.values) + ')', tuple(self.types))
    
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
        types = [Joint.PRISMATIC] * len(prismatic_values) + [Joint.REVOLUTE] * len(revolute_values)
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
            'types': self.types
        }

    @data.setter
    def data(self, data):
        self.values = data.get('values') or []
        self.types = data.get('types') or []

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
        return cls(self.values[:], self.types[:])


if __name__ == "__main__":
    from math import pi
    from math import radians

    q = [4.5, 1.7, 0.5, 2.1, 0.1, 2.1]
    configuration = Configuration.from_revolute_values(q)
    print(configuration)

    config = Configuration.from_revolute_values([pi/2, 0., 0.])
    print(config.values)

    config = Configuration.from_prismatic_and_revolute_values([8.312], [pi/2, 0., 0., 0., 2*pi, 0.8])
    print(str(config))

    config = Configuration([pi/2, 3., 0.1], [Joint.REVOLUTE, Joint.PRISMATIC, Joint.PLANAR])
    print(str(config))

    config = Configuration.from_prismatic_and_revolute_values([2., 3., 1.33], map(radians, [90, 0, 0, 20]))
    print(str(config))
    print(config.revolute_values)
    print(config.prismatic_values)
