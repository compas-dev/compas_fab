from __future__ import print_function

import math

import compas
from compas.geometry import Vector
from compas.geometry import cross_vectors

if not compas.is_ironpython():
    from scipy import stats
else:
    stats = None

# TODO: move to some constants file? g is also defined in scipy.constants
# standard acceleration of gravity [m/s**2]
g = 9.80665
gravity_vector = Vector(0, 0, -g)


__all__ = ['Wrench']


class Wrench():
    """A wrench represents force in free space, separated into its linear (force) and angular (torque) parts.

    Attributes
    ----------
    force : :class:`Vector`
        [Fx, Fy, Fz] force vector in Newtons
    torque : :class:`Vector`
        [Tx, Ty, Tz] moments vector in Newton-meters

    Examples
    --------
    >>> from compas.geometry import Vector
    >>> w = Wrench([1, 2, 3], [0.1, 0.2, 0.3])
    >>> w = Wrench(Vector(1, 2, 3), Vector(0.1, 0.2, 0.3))

    """

    def __init__(self, force, torque):
        self.force = force
        self.torque = torque

    # ==========================================================================
    # factory
    # ==========================================================================

    @classmethod
    def from_list(cls, values):
        """Construct a wrench from a list of 6 :obj:`float` values.

        Parameters
        ----------
        values : :obj:`list` of :obj:`float`
            The list of 6 values representing a wrench.

        Returns
        -------
        :class:`Wrench`
            The constructed wrench.

        Examples
        --------
        >>> w = Wrench.from_list([1, 2, 3, 0.1, 0.2, 0.3])
        """
        force = values[0:3]
        torque = values[3:6]
        return cls(force, torque)

    @classmethod
    def from_data(cls, data):
        """Construct a wrench from its data representation.

        Parameters
        ----------
        data : :obj:`dict`
            The data dictionary.

        Returns
        -------
        :class:`Wrench`
            The constructed wrench.

        Examples
        --------
        >>> data = {"force": [1, 2, 3], "torque": [0.1, 0.2, 0.3]}
        >>> w = Wrench.from_data(data)
        """
        force = data["force"]
        torque = data["torque"]
        return cls(force, torque)

    @classmethod
    def by_samples(cls, wrenches, proportion_to_cut=0.1):
        """
        Construct the wrench by sampled data, allowing to filter.

        Parameters
        ----------
        wrenches : list of :class:`Wrench`
            List of wrenches.
        proportion_to_cut : :obj:`float`
            Fraction to cut off of both tails of the distribution

        Returns
        -------
        Wrench
            The mean wrench after trimming distribution from both tails.

        Examples
        --------
        >>> w1 = Wrench([1, 1, 1], [.1,.1,.1])
        >>> w2 = Wrench([2, 2, 2], [.2,.2,.2])
        >>> w3 = Wrench([3, 3, 3], [.3,.3,.3])
        >>> w = Wrench.by_samples([w1, w2, w3])
        >>> print(w)
        Wrench(Vector(2.000, 2.000, 2.000), Vector(0.200, 0.200, 0.200))
        """
        if not stats:
            raise NotImplementedError("Not supported on this platform")

        forces = [w.force for w in wrenches]
        torques = [w.torque for w in wrenches]
        force = stats.trim_mean(forces, proportion_to_cut, axis=0).tolist()
        torque = stats.trim_mean(torques, proportion_to_cut, axis=0).tolist()
        return cls(force, torque)

    # ==========================================================================
    # descriptors
    # ==========================================================================

    @property
    def force(self):
        return self._force

    @force.setter
    def force(self, vector):
        force = Vector(*list(vector))
        self._force = force

    @property
    def torque(self):
        return self._torque

    @torque.setter
    def torque(self, vector):
        torque = Vector(*list(vector))
        self._torque = torque

    @property
    def data(self):
        """Returns the data dictionary that represents the wrench.

        Returns
        -------
        dict
            The wrench data.
        """
        return {'force': list(self.force),
                'torque': list(self.torque)}

    def to_data(self):
        """Returns the data dictionary that represents the wrench.

        Returns
        -------
        dict
            The wrench data.
        """
        return self.data

    # ==========================================================================
    # access
    # ==========================================================================

    def __iter__(self):
        return iter(list(self.force) + list(self.torque))

    def __getitem__(self, item):
        w = list(self)
        return w[item]

    # ==========================================================================
    # representation
    # ==========================================================================

    def __repr__(self):
        return "Wrench({0}, {1})".format(self.force, self.torque)

    # ==========================================================================
    # helpers
    # ==========================================================================

    def copy(self):
        """Make a copy of this ``Wrench``.

        Returns
        -------
        Wrench
            The copy.
        """
        cls = type(self)
        return cls(self.force.copy(), self.torque.copy())

    # ==========================================================================
    # operators
    # ==========================================================================

    def __mul__(self, n):
        """Create a ``Wrench`` multiplied by the given factor.

        Parameters
        ----------
        n : float
            The multiplication factor.

        Returns
        -------
        Wrench
            The resulting new ``Wrench``.

        """
        return Wrench(self.force * n, self.torque * n)

    def __add__(self, other):
        """Return a ``Wrench`` that is the the sum of this ``Wrench`` and another wrench.

        Parameters
        ----------
        other : wrench
            The wrench to add.

        Returns
        -------
        Wrench
            The resulting new ``Wrench``.

        """
        return Wrench(
            self.force + other.force,
            self.torque + other.torque)

    def __sub__(self, other):
        """Return a ``Wrench`` that is the the difference between this ``Wrench`` and another wrench.

        Parameters
        ----------
        other : wrench
            The wrench to subtract.

        Returns
        -------
        Wrench
            The resulting new ``Wrench``.
        """
        return Wrench(
            self.force - other.force,
            self.torque - other.torque)

    def __neg__(self):
        """Return the negated ``Wrench``.

        Returns
        -------
        Wrench
            The negated ``Wrench``.
        """
        return Wrench(
            self.force * -1.0,
            self.torque * -1.0)

    def __len__(self):
        return 6

    # ==========================================================================
    # comparison
    # ==========================================================================

    def __eq__(self, other, tol=1e-05):
        for a, b in zip(list(self), list(other)):
            if math.fabs(a - b) > tol:
                return False
        return True

    def __ne__(self, other, tol=1e-05):
        return not self.__eq__(other, tol)

    # ==========================================================================
    # transformations
    # ==========================================================================

    def transform(self, transformation):
        """Transforms a `Wrench` with the transformation.

        Parameters
        ----------
        transformation : :class:`Transformation`
            The transformation to transform the `Wrench`.

        Returns
        -------
        None

        Examples
        --------
        >>> R = Rotation.from_axis_and_angle([1, 0, 0], math.pi)
        >>> w = Wrench([1, 2, 3], [0.1, 0.2, 0.3])
        >>> w.transform(R)
        """
        self.force.transform(transformation)
        self.torque.transform(transformation)

    def transformed(self, transformation):
        """Returns a transformed copy of the `Wrench`.

        Parameters
        ----------
        transformation : :class:`Transformation`
            The transformation to transform the `Wrench`.

        Returns
        -------
        Wrench
            The transformed wrench.

        Examples
        --------
        >>> R = Rotation.from_axis_and_angle([1, 0, 0], math.pi)
        >>> w1 = Wrench([1, 2, 3], [0.1, 0.2, 0.3])
        >>> w2 = w1.transformed(R)
        """
        wrench = self.copy()
        wrench.transform(transformation)
        return wrench

    def gravity_compensated(self, ft_sensor_frame, mass, center_of_mass):
        """Removes the force and torque in effect of gravity from the wrench.

        Parameters
        ----------
        ft_sensor_frame : :class:`compas.geometry.Frame`
            The coordinate frame of the force torque sensor.
        mass: float
            The mass of the object in kg.
        center_of_mass : :class:`Point`
            The center of mass of the object in meters.

        Returns
        -------
        Wrench
            The gravity compensated wrench.

        Examples
        --------
        >>> mass, com = 10, [0, 0, 1]
        >>> f = Frame([0, 0, 0], [1, 0, 0], [0, 1, 0])
        >>> w = Wrench([0, 0, -98], [0, 0, 0])
        >>> w.gravity_compensated(f, mass, com)
        Wrench(Vector(0.000, 0.000, 0.066), Vector(0.000, 0.000, 0.000))

        >>> mass, com = 10, [1, 1, 1]
        >>> f = Frame([0, 0, 0], [1, 0, 0], [0, 1, 0])
        >>> w = Wrench([0, 0, -98], [-98, 98, 0])
        >>> w.gravity_compensated(f, mass, com)
        Wrench(Vector(0.000, 0.000, 0.066), Vector(-88.193, 88.193, 0.000))

        Notes
        -----
        For more info, see [1]_.

        References
        ----------
        .. [1] Vougioukas S., *Bias Estimation and Gravity Compensation For
            Force-Torque Sensors*,
            Available at: http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.552.109.
        """
        # transform gravity vector to FT Sensor coordinate system (FTSCS)
        gravity_vector_FTSCS = ft_sensor_frame.to_local_coordinates(gravity_vector)

        # F gravity compensation, F = gravity * mass
        force_gravity = gravity_vector_FTSCS * mass

        # torque gravity compensation, T = (lever_arm * mass) X gravity_vector_FTSCS
        torque_gravity = Vector(*cross_vectors((center_of_mass * mass), gravity_vector_FTSCS))

        return Wrench(self.force - force_gravity, self.torque - torque_gravity)
