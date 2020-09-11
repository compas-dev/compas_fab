from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from math import pi
from compas.robots import Joint

__all__ = [
    'Configuration',
]


class Configuration(object):
    """Represents the configuration of a robot based on the state of its joints.

    This concept is also refered to as \"Joint State\".

    Parameters
    ----------
    values : :obj:`list` of :obj:`float`
        Joint values expressed in radians or meters, depending on the respective
        type.
    types : :obj:`list` of :attr:`compas.robots.Joint.SUPPORTED_TYPES`
        Joint types, e.g. a list of :attr:`compas.robots.Joint.REVOLUTE` for
        revolute joints.
    joint_names : :obj:`list` of :obj:`str`, optional
        List of joint names.

    Attributes
    ----------
    values : :obj:`list` of :obj:`float`
        Joint values expressed in radians or meters, depending on the respective
        type.
    types : :obj:`list` of :attr:`compas.robots.Joint.SUPPORTED_TYPES`
        Joint types, e.g. a list of :attr:`compas.robots.Joint.REVOLUTE` for
        revolute joints.
    joint_names : :obj:`list` of :obj:`str`
        List of joint names.
    data : :obj:`dict`
        The data representing the configuration.
    prismatic_values : :obj:`list` of :obj:`float`
        Prismatic joint values in meters.
    revolute_values : :obj:`list` of :obj:`float`
        Revolute joint values in radians.

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
        """Return a human-readable string representation of the instance."""
        v_str = ('(' + ", ".join(['%.' + self._precision] * len(self.values)) + ')') % tuple(self.values)
        if len(self.joint_names):
            return "Configuration({}, {}, {})".format(v_str, tuple(self.types), tuple(self.joint_names))
        else:
            return "Configuration({}, {})".format(v_str, tuple(self.types))

    def __repr__(self):
        """Printable representation of :class:`Configuration`."""
        return self.__str__()

    @classmethod
    def from_revolute_values(cls, values, joint_names=None):
        """Construct a configuration from revolute joint values in radians.

        Parameters
        ----------
        values : :obj:`list` of :obj:`float`
            Joint values expressed in radians.
        joint_names : :obj:`list` of :obj:`str`, optional
            List of joint names.

        Returns
        -------
        :class:`Configuration`
             An instance of :class:`Configuration` instance.
        """
        values = list(values)
        joint_names = list(joint_names or [])
        return cls.from_data({'values': values, 'types': [Joint.REVOLUTE] * len(values), 'joint_names': joint_names})

    @classmethod
    def from_prismatic_and_revolute_values(cls, prismatic_values, revolute_values, joint_names=None):
        """Construct a configuration from prismatic and revolute joint values.

        Parameters
        ----------
        prismatic_values : :obj:`list` of :obj:`float`
            Positions on the external axis system in meters.
        revolute_values : :obj:`list` of :obj:`float`
            Joint values expressed in radians.
        joint_names : :obj:`list` of :obj:`str`, optional
            List of joint names.

        Returns
        -------
        :class:`Configuration`
             An instance of :class:`Configuration`.
        """
        # Force iterables into lists
        prismatic_values = list(prismatic_values)
        revolute_values = list(revolute_values)
        joint_names = list(joint_names or [])
        values = prismatic_values + revolute_values
        types = [Joint.PRISMATIC] * \
            len(prismatic_values) + [Joint.REVOLUTE] * len(revolute_values)
        return cls.from_data({'values': values, 'types': types, 'joint_names': joint_names})

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
             An instance of :class:`Configuration`.
        """
        config = cls()
        config.data = data
        return config

    def to_data(self):
        """Get the data dictionary that represents the configuration.

        This data can also be used to reconstruct the :class:`Configuration`
        instance.

        Returns
        -------
        :obj:`dict`
            The data representing the configuration.
        """
        return self.data

    @property
    def data(self):
        """:obj:`dict` : The data representing the configuration.

        By assigning a data dictionary to this property, the current data of
        the configuration will be replaced by the data in the :obj:`dict`. The
        data getter and setter should always be used in combination with each
        other.
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

        E.g. positions on the external axis system.
        """
        return [v for i, v in enumerate(self.values) if self.types[i] == Joint.PRISMATIC]

    @property
    def revolute_values(self):
        """:obj:`list` of :obj:`float` : Revolute joint values in radians."""
        return [v for i, v in enumerate(self.values) if self.types[i] == Joint.REVOLUTE]

    def copy(self):
        """Create a copy of this :class:`Configuration`.

        Returns
        -------
        :class:`Configuration`
            An instance of :class:`Configuration`
        """
        cls = type(self)
        return cls(self.values[:], self.types[:], self.joint_names[:])

    def scale(self, scale_factor):
        """Scales the joint positions of the current configuration.

        Only scalable joints are scaled, i.e. planar and prismatic joints.

        Parameters
        ----------
        scale_factor : :obj:`float`
            Scale factor.

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
        """Return a scaled copy of this configuration.

        Only scalable joints are scaled, i.e. planar and prismatic joints.

        Parameters
        ----------
        scale_factor : :obj:`float`
            Scale factor

        Returns
        -------
        None
        """
        config = self.copy()
        config.scale(scale_factor)
        return config

    def differences_generator(self, other):
        """Generator over the differences to another `Configuration`'s values.

        If the joint type is revolute or continuous, the smaller difference
        (+/- 2*:math:`\\pi`) is calculated.

        Parameters
        ----------
        other : :class:`Configuration`
            The configuration to compare to.

        Yields
        ------
        :obj:`float`
            The next difference to the `Configuration`'s values.

        Raises
        ------
        ValueError
            If the configurations are not comparable.

        Examples
        --------
        >>> c1 = Configuration.from_revolute_values([1, 0, 3])
        >>> c2 = Configuration.from_revolute_values([1, 2 * pi, 4])
        >>> list(c1.differences_generator(c2))
        [0, 0.0, -1]
        """
        if len(self.joint_names) and len(other.joint_names):
            d1 = dict(zip(self.joint_names, self.values))
            d2 = dict(zip(other.joint_names, other.values))
            types = dict(zip(self.joint_names, self.types))
            for k, v in d1.items():
                if k not in d2:
                    raise ValueError("Configurations have different joint names.")
                diff = v - d2[k]
                if types[k] in [Joint.REVOLUTE, Joint.CONTINUOUS]:
                    sign = -1 if diff > 0 else 1
                    while abs(diff) > abs(diff + sign * 2 * pi):
                        diff = diff + sign * 2 * pi
                yield diff
        else:
            if len(self.values) != len(other.values):
                raise ValueError("Can't compare configurations with different lengths of values.")
            for i, (v1, v2) in enumerate(zip(self.values, other.values)):
                diff = v1 - v2
                if self.types[i] in [Joint.REVOLUTE, Joint.CONTINUOUS]:
                    sign = -1 if diff > 0 else 1
                    while abs(diff) > abs(diff + sign * 2 * pi):
                        diff = diff + sign * 2 * pi
                yield diff

    def max_difference(self, other):
        """Returns the maximum difference to another `Configuration`'s values.

        Parameters
        ----------
        other : :class:`Configuration`
            The configuration to compare to.

        Returns
        ------
        :obj:`float`
            The maximum absolute difference.

        Examples
        --------
        >>> c1 = Configuration.from_revolute_values([1, 0, 3])
        >>> c2 = Configuration.from_revolute_values([1, 2 * pi, 4])
        >>> c1.max_difference(c2)
        1
        """
        return max([abs(v) for v in self.differences_generator(other)])

    def close_to(self, other, tol=1e-3):
        """Returns ``True`` if the other `Configuration`'s values are within a certain range.

        Parameters
        ----------
        other : :class:`Configuration`
            The configuration to compare to.
        tol : float
            The tolerance under which we consider 2 floats the same. Defaults to 1e-3.

        Returns
        -------
        :obj:`bool`
            ``True`` if the other `Configuration`'s values are within a certain
            tolerance, `False` otherwise.

        Examples
        --------
        >>> c1 = Configuration.from_revolute_values([1, 0, 3])
        >>> c2 = Configuration.from_revolute_values([1, 2 * pi, 3])
        >>> c1.close_to(c2)
        True
        """
        for diff in self.differences_generator(other):
            if abs(diff) > tol:
                return False
        return True


if __name__ == "__main__":
    import math  # noqa F401
    import doctest
    doctest.testmod(globs=globals())
