import math

from compas.data import Data
from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Transformation
from compas.geometry import Vector
from compas.geometry import cross_vectors

try:
    from scipy import stats
except ImportError:
    stats = None

# TODO: move to some constants file? g is also defined in scipy.constants
# standard acceleration of gravity [m/s**2]
G = 9.80665
GRAVITY_VECTOR = Vector(0, 0, -G)


__all__ = ["Wrench"]


class Wrench(Data):
    """A wrench represents force in free space, separated into its linear (force) and angular (torque) parts.

    Attributes
    ----------
    force : `Vector`
        [Fx, Fy, Fz] force vector in Newtons
    torque : `Vector`
        [Tx, Ty, Tz] moments vector in Newton-meters

    Examples
    --------
    >>> from compas.geometry import Vector
    >>> w = Wrench([1, 2, 3], [0.1, 0.2, 0.3])
    >>> w = Wrench(Vector(1, 2, 3), Vector(0.1, 0.2, 0.3))

    """

    def __init__(self, force: Vector, torque: Vector):
        super(Wrench, self).__init__()
        self.force = force
        self.torque = torque

    @property
    def __data__(self):
        """Returns the data dictionary that represents the wrench.

        Returns
        -------
        dict
            The wrench data.
        """
        return {"force": self.force.__data__, "torque": self.torque.__data__}

    # ==========================================================================
    # factory
    # ==========================================================================

    @classmethod
    def from_list(cls, values: list[float]) -> "Wrench":
        """Construct a wrench from a list of 6 `float` values.

        Parameters
        ----------
        values : `list` of `float`
            The list of 6 values representing a wrench.

        Returns
        -------
        `Wrench`
            The constructed wrench.

        Examples
        --------
        >>> w = Wrench.from_list([1, 2, 3, 0.1, 0.2, 0.3])
        """
        force = values[0:3]
        torque = values[3:6]
        return cls(force, torque)

    @classmethod
    def by_samples(cls, wrenches: list["Wrench"], proportion_to_cut: float = 0.1) -> "Wrench":
        """
        Construct the wrench by sampled data, allowing to filter.

        Parameters
        ----------
        wrenches : list of `Wrench`
            List of wrenches.
        proportion_to_cut : `float`
            Fraction to cut off of both tails of the distribution

        Returns
        -------
        Wrench
            The mean wrench after trimming distribution from both tails.

        Examples
        --------
        >>> w1 = Wrench([1, 1, 1], [0.1, 0.1, 0.1])
        >>> w2 = Wrench([2, 2, 2], [0.2, 0.2, 0.2])
        >>> w3 = Wrench([3, 3, 3], [0.3, 0.3, 0.3])
        >>> w = Wrench.by_samples([w1, w2, w3])
        >>> print(w.force)
        Vector(x=2.000, y=2.000, z=2.000)
        >>> print(w.torque)
        Vector(x=0.200, y=0.200, z=0.200)
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
    def force(self) -> Vector:
        return self._force

    @force.setter
    def force(self, vector: Vector):
        force = Vector(*list(vector))
        self._force = force

    @property
    def torque(self) -> Vector:
        return self._torque

    @torque.setter
    def torque(self, vector: Vector):
        torque = Vector(*list(vector))
        self._torque = torque

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
        return "Wrench({!r}, {!r})".format(self.force, self.torque)

    # ==========================================================================
    # helpers
    # ==========================================================================

    def copy(self) -> "Wrench":
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

    def __mul__(self, n: float) -> "Wrench":
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

    def __add__(self, other: "Wrench") -> "Wrench":
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
        return Wrench(self.force + other.force, self.torque + other.torque)

    def __sub__(self, other: "Wrench") -> "Wrench":
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
        return Wrench(self.force - other.force, self.torque - other.torque)

    def __neg__(self) -> "Wrench":
        """Return the negated ``Wrench``.

        Returns
        -------
        Wrench
            The negated ``Wrench``.
        """
        return Wrench(self.force * -1.0, self.torque * -1.0)

    def __len__(self) -> int:
        return 6

    # ==========================================================================
    # comparison
    # ==========================================================================

    def __eq__(self, other: "Wrench", tol: float = 1e-05) -> bool:
        for a, b in zip(list(self), list(other)):
            if math.fabs(a - b) > tol:
                return False
        return True

    def __ne__(self, other: "Wrench", tol: float = 1e-05) -> bool:
        return not self.__eq__(other, tol)

    # ==========================================================================
    # transformations
    # ==========================================================================

    def transform(self, transformation: Transformation) -> None:
        """Transforms a `Wrench` with the transformation.

        Parameters
        ----------
        transformation : `Transformation`
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

    def transformed(self, transformation: Transformation) -> "Wrench":
        """Returns a transformed copy of the `Wrench`.

        Parameters
        ----------
        transformation : `Transformation`
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

    def gravity_compensated(self, ft_sensor_frame: Frame, mass: float, center_of_mass: Point) -> "Wrench":
        """Removes the force and torque in effect of gravity from the wrench.

        Parameters
        ----------
        ft_sensor_frame : [`Frame`][compas.geometry.Frame]
            The coordinate frame of the force torque sensor.
        mass : float
            The mass of the object in kg.
        center_of_mass : [`Point`][compas.geometry.Point]
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
        >>> x = w.gravity_compensated(f, mass, com)
        >>> print(x.force)
        Vector(x=0.000, y=0.000, z=0.066)
        >>> print(x.torque)
        Vector(x=0.000, y=0.000, z=0.000)

        >>> mass, com = 10, [1, 1, 1]
        >>> f = Frame([0, 0, 0], [1, 0, 0], [0, 1, 0])
        >>> w = Wrench([0, 0, -98], [-98, 98, 0])
        >>> x = w.gravity_compensated(f, mass, com)
        >>> print(x.force)
        Vector(x=0.000, y=0.000, z=0.066)
        >>> print(x.torque)
        Vector(x=-88.193, y=88.193, z=0.000)

        Notes
        -----
        For more info, see [1]_.

        References
        ----------
        .. [1] Vougioukas S., *Bias Estimation and Gravity Compensation For
            Force-Torque Sensors*,
            Available at: https://www.semanticscholar.org/paper/Bias-Estimation-and-Gravity-Compensation-For-Vougioukas/900c5de4ac54cf28df816584264fa0de71c4817f

        """
        # transform gravity vector to FT Sensor coordinate system (FTSCS)
        gravity_vector_FTSCS = ft_sensor_frame.to_local_coordinates(GRAVITY_VECTOR)

        # F gravity compensation, F = gravity * mass
        force_gravity = gravity_vector_FTSCS * mass

        # torque gravity compensation, T = (lever_arm * mass) X gravity_vector_FTSCS
        torque_gravity = Vector(*cross_vectors((center_of_mass * mass), gravity_vector_FTSCS))

        return Wrench(self.force - force_gravity, self.torque - torque_gravity)
