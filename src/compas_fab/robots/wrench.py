from __future__ import print_function

import math

import compas
from compas.geometry import Frame
from compas.geometry import Transformation
from compas.geometry import Vector
from compas.geometry import cross_vectors

if not compas.is_ironpython():
    from scipy import stats
else:
    stats = None


__all__ = ['Wrench']


class Wrench():
    """
    A wrench is defined by a vector describing the 3 components of force, and
    another vector describing the 3 components of torque.
    Optional values for initialization are the rigid body inertia.

    Attributes
    ----------
    force : vector
        [Fx,Fy,Fz] force vector in Newtons
    torque : vector
        [Tx,Ty,Tz] moments vector in Newton-meters
    inertia : inertia object

    Examples
    --------
    >>> from compas.geometry import Vector
    >>> w = Wrench([1, 2, 3], [0.1, 0.2, 0.3])
    >>> w = Wrench(Vector(1, 2, 3), Vector(0.1, 0.2, 0.3))
    """

    def __init__(self, force, torque, inertia=None):
        self._force = None
        self._torque = None
        self.force = force
        self.torque = torque
        self.inertia = inertia

    # ==========================================================================
    # factory
    # ==========================================================================

    @classmethod
    def from_list(cls, values, inertia=None):
        """Construct a wrench from a list of 6 :obj:`float` values.

        Parameters
        ----------
        values : :obj:`list` of :obj:`float`
            The list of 6 values representing a wrench.

        Returns
        -------
        Wrench
            The constructed wrench.

        Examples
        --------
        >>> w = Wrench.from_list([1, 2, 3, 0.1, 0.2, 0.3])
        """
        force = values[0:3]
        torque = values[3:6]
        return cls(force, torque, inertia=inertia)

    @classmethod
    def from_data(cls, data, inertia=None):
        """Construct a wrench from its data representation.

        Parameters
        ----------
        data : :obj:`dict`
            The data dictionary.

        Returns
        -------
        Wrench
            The constructed wrench.

        Examples
        --------
        >>> data = {"force": [1, 2, 3], "torque": [0.1, 0.2, 0.3]}
        >>> w = Wrench.from_data(data)
        """
        force = data["force"]
        torque = data["torque"]
        return cls(force, torque, inertia=inertia)

    @classmethod
    def by_samples(cls, wrenches, inertia=None, proportiontocut=0.1):
        """
        Construct the wrench by sampled data, allowing to filter.

        Parameters
        ----------
        forces : :obj:`list` of :obj:`list` of :obj:`float`
            List of forces.
        torques : :obj:`list` of :obj:`list` of :obj:`float`
            List of torques.
        proportiontocut : :obj:`float`
            Fraction to cut off of both tails of the distribution

        Returns
        -------
        Wrench
            The mean wrench after trimming distribution from both tails.

        Examples
        --------
        >>> w1 = Wrench([1,1,1], [.1,.1,.1])
        >>> w2 = Wrench([2,2,2], [.2,.2,.2])
        >>> w3 = Wrench([3,3,3], [.3,.3,.3])
        >>> w = Wrench.by_samples([w1,w2,w3])
        >>> print(w)
        Wrench(Vector(2.000, 2.000, 2.000), Vector(0.200, 0.200, 0.200))
        """
        if not stats:
            raise NotImplementedError("Not supported on this platform")

        forces = [w.force for w in wrenches]
        torques = [w.torque for w in wrenches]
        force = stats.trim_mean(forces, proportiontocut, axis=0).tolist()
        torque = stats.trim_mean(torques, proportiontocut, axis=0).tolist()
        return cls(force, torque, inertia=inertia)

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
        if self.inertia:
            return "Wrench({0}, {1}, {2})".format(self.force, self.torque, self.inertia)
        else:
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
        return cls(self.force.copy(), self.torque.copy(), inertia=self.inertia)

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
        return Wrench(self.force * n, self.torque * n, inertia=self.inertia)

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
            self.torque + other.torque,
            inertia=self.inertia)

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
            self.torque - other.torque,
            inertia=self.inertia)

    def __neg__(self):
        """Return the negated ``Wrench``.

        Returns
        -------
        Wrench
            The negated ``Wrench``.
        """
        return Wrench(
            self.force * -1.0,
            self.torque * -1.0,
            inertia=self.inertia)

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

    # ==========================================================================
    # transformations
    # ==========================================================================

    def transform(self, matrix):
        """Transforms a wrench given a transformation matrix.

        Parameters
        ----------
        matrix : list of list
            The transformation matrix.

        Examples
        --------
        >>> t = [[1,0,0,0], [0,-1,0,0], [0,0,-1,0], [0,0,0,1]]
        >>> w = Wrench([1, 2, 3], [0.1, 0.2, 0.3])
        >>> w.transform(t)
        """
        self.force.transform(matrix)
        self.torque.transform(matrix)

    def transformed(self, matrix):
        """Returns a transformed copy of the wrench.

        Parameters
        ----------
        matrix : list of list
            The transformation matrix.

        Returns
        -------
        Wrench
            The transformed wrench.

        Examples
        --------
        >>> t = [[1,0,0,0], [0,-1,0,0], [0,0,-1,0], [0,0,0,1]]
        >>> w1 = Wrench([1, 2, 3], [0.1, 0.2, 0.3])
        >>> w2 = w1.transformed(t)
        """
        wrench = self.copy()
        wrench.transform(matrix)
        return wrench

    def gravity_compensated(self, frame):
        """
        Removes the force and torque in effect of gravity from the wrench.

        Parameters
        ----------
        frame : :class:`compas.geometry.Frame`
            A frame.

        Returns
        -------
        Wrench
            The gravity compensated wrench.

        Examples
        --------
        >>> from compas.geometry import Frame
        >>> from inertia import Inertia
        >>> i = Inertia(10, [0,0,1])
        >>> f = Frame([0,0,0], [1,0,0], [0,1,0])
        >>> w = Wrench([0,0,-98], [0,0,0], inertia=i)
        >>> w.gravity_compensated(f)
        Wrench(Vector(0.000, 0.000, 0.000), Vector(0.000, 0.000, 0.000), Mass(10), CoM(Vector(0.000, 0.000, 1.000)), Tensor(None), Gravity magnitude(9.8))
        >>> i = Inertia(10, [1,1,1])
        >>> f = Frame([0,0,0], [1,0,0], [0,1,0])
        >>> w = Wrench([0,0,-98], [-98,98,0], inertia=i)
        >>> w.gravity_compensated(f)
        Wrench(Vector(0.000, 0.000, 0.000), Vector(0.000, 0.000, 0.000), Mass(10), CoM(Vector(1.000, 1.000, 1.000)), Tensor(None), Gravity magnitude(9.8))
        """
        if frame and self.inertia:
            # transform gravity vector to FT Sensor coordinate system (FTSCS)
            FTSCS_transformation = Transformation.from_frame_to_frame(frame, Frame.worldXY())
            gravity_vector_FTSCS = self.inertia.gravity_vector_WCS.transformed(FTSCS_transformation)

            # F gravity compensation
            # F = gravity * mass
            F_gravity = gravity_vector_FTSCS * self.inertia.mass

            # T gravity compensation
            # T = (lever_arm * mass) X gravity_vector_FTSCS
            T_gravity = Vector(*cross_vectors((self.inertia.center_of_mass * self.inertia.mass), gravity_vector_FTSCS))

            force = self.force - F_gravity
            torque = self.torque - T_gravity
            return Wrench(force, torque, inertia=self.inertia)
        else:
            return None


# ==============================================================================
# Main
# ==============================================================================
if __name__ == "__main__":

    import doctest
    doctest.testmod()
