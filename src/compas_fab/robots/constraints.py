from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from compas.geometry import Scale
from compas.geometry import Rotation

__all__ = ['BoundingVolume', 'Constraint', 'JointConstraint',
           'OrientationConstraint', 'PositionConstraint']


class BoundingVolume(object):
    """A container for describing a bounding volume.

    Attributes
    ----------
    type: int
        The type of the bounding volume.
    volume: object
        The volume can be either a `Box`, a `Sphere`, or a `Mesh`.
    """

    BOX = 1
    SPHERE = 2
    MESH = 3

    def __init__(self, type, volume):
        if type not in [self.BOX, self.SPHERE, self.MESH]:
            raise ValueError("Type must be %d, %d or %d"
                             % (self.BOX, self.SPHERE, self.MESH))
        self.type = type
        self.volume = volume

    @classmethod
    def from_box(cls, box):
        """Creates a `BoundingVolume` from a :class:`compas.geometry.Box`.

        Parameters
        ----------
        box: `compas.geometry.Box`

        Examples
        --------
        >>> from compas.geometry import Frame
        >>> from compas.geometry import Box
        >>> from compas_fab.robots import BoundingVolume
        >>> box = Box(Frame.worldXY(), 1., 1., 1.)
        >>> bv = BoundingVolume.from_box(box)
        >>> bv.type
        1
        """
        return cls(cls.BOX, box)

    @classmethod
    def from_sphere(cls, sphere):
        """Creates a ``BoundingVolume`` from a :class:`compas.geometry.Sphere`.

        Parameters
        ----------
        sphere: `compas.geometry.Sphere`

        Examples
        --------
        >>> from compas.geometry import Sphere
        >>> from compas_fab.robots import BoundingVolume
        >>> sphere = Sphere((1., 1., 1.), 5.)
        >>> bv = BoundingVolume.from_sphere(sphere)
        >>> bv.type
        2
        """
        return cls(cls.SPHERE, sphere)

    @classmethod
    def from_mesh(cls, mesh):
        """Creates a ``BoundingVolume`` from a :class:`compas.datastructures.Mesh`.

        Parameters
        ----------
        mesh: `compas.datastructures.Mesh`

        Examples
        --------
        >>> import compas
        >>> from compas.datastructures import Mesh
        >>> from compas_fab.robots import BoundingVolume
        >>> mesh = Mesh.from_obj(compas.get('faces.obj'))
        >>> bv = BoundingVolume.from_mesh(Mesh)
        >>> bv.type
        3
        """
        return cls(cls.MESH, mesh)

    def scale(self, scale_factor):
        S = Scale([1. / scale_factor] * 3)
        self.transform(S)

    def transform(self, transformation):
        self.volume.transform(transformation)

    def __repr__(self):
        return "BoundingVolume({0}, {1})".format(self.type, self.volume)

    def copy(self):
        """Make a copy of this ``BoundingVolume``.

        Returns
        -------
        BoundingVolume
            The copy.

        """
        cls = type(self)
        return cls(self.type, self.volume.copy())


class Constraint(object):
    """Base class for robot constraints.

    Attributes
    ----------
    weight: float, optional
        A weighting factor for this constraint. Denotes relative importance to
        other constraints. Closer to zero means less important. Defaults to 1.
    """

    JOINT = 1
    POSITION = 2
    ORIENTATION = 3
    possible_types = (JOINT, POSITION, ORIENTATION)

    def __init__(self, type, weight=1.):
        if type not in self.possible_types:
            raise ValueError("Type must be %d, %d or %d" % self.possible_types)
        self.type = type
        self.weight = weight

    def transform(self, transformation):
        pass

    def scale(self, scale_factor):
        pass

    def copy(self):
        cls = type(self)
        return cls(self.type, self.weight)


class JointConstraint(Constraint):
    """Constrains the value of a joint to be within a certain bound.

    Attributes
    ----------
    joint_name: string
        The name of the joint this contraint refers to.
    value: float
        The targeted value for that joint.
    tolerance_above: float
        Tolerance above the targeted joint value, in radians. Defaults to 0.
    tolerance_below: float
        Tolerance below the targeted joint value, in radians. Defaults to 0.
        The bound to be achieved is [value - tolerance_below, value + tolerance_above].
    weight: float, optional
        A weighting factor for this constraint. Denotes relative importance to
        other constraints. Closer to zero means less important. Defaults to 1.

    Examples
    --------
    >>> from compas_fab.robots import JointConstraint
    >>> jc = JointConstraint("joint_0", 1.4, 0.1, 0.1, 1.0)

    """

    def __init__(self, joint_name, value, tolerance_above=0., tolerance_below=0., weight=1.):
        super(JointConstraint, self).__init__(self.JOINT, weight)
        self.joint_name = joint_name
        self.value = value
        self.tolerance_above = abs(tolerance_above)
        self.tolerance_below = abs(tolerance_below)

    def scale(self, scale_factor):
        self.value /= scale_factor
        self.tolerance_above /= scale_factor
        self.tolerance_below /= scale_factor

    def __repr__(self):
        return "JointConstraint('{0}', {1}, {2}, {3}, {4})".format(self.joint_name, self.value, self.tolerance_above, self.tolerance_below, self.weight)

    def copy(self):
        cls = type(self)
        return cls(self.joint_name, self.value, self.tolerance_above, self.tolerance_below, self.weight)


class OrientationConstraint(Constraint):
    """Constrains a link to be within a certain orientation.

    Attributes
    ----------
    link_name: string
        The name of the link this contraint refers to.
    quaternion: list of float
        The desired orientation of the link specified by a quaternion in the
        order of [w, x, y, z].
    tolerances: list of float, optional
        Error tolerances ti for each of the frame's axes. The respective
        bound to be achieved is [ai - ti, ai + ti]. Defaults to [0.01, 0.01, 0.01].
    weight: float, optional
        A weighting factor for this constraint. Denotes relative importance to
        other constraints. Closer to zero means less important. Defaults to 1.

    Notes
    -----
    If you specify the tolerance vector with [0.01, 0.01, 6.3], it means that
    the frame's x-axis and y-axis are allowed to rotate about the z-axis by an
    angle of 6.3 radians, whereas the z-axis can only change 0.01.

    Examples
    --------
    >>> from compas.geometry import Frame
    >>> from compas_fab.robots import OrientationConstraint
    >>> frame = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
    >>> oc = OrientationConstraint("link_0", frame.quaternion)

    """

    def __init__(self, link_name, quaternion, tolerances=None, weight=1.):
        super(OrientationConstraint, self).__init__(self.ORIENTATION, weight)
        self.link_name = link_name
        self.quaternion = [float(a) for a in list(quaternion)]
        self.tolerances = [float(a) for a in list(tolerances)] if tolerances else [0.01] * 3

    def transform(self, transformation):
        R = Rotation.from_quaternion(self.quaternion)
        R = transformation * R

        # Due to a bug on COMPAS 0.10.0
        # (Fixed on https://github.com/compas-dev/compas/pull/378 but not released atm)
        # we work around the retrival of the rotation component of R and instead decompose and get it
        _, _, r, _, _ = R.decomposed()
        self.quaternion = r.quaternion
        # After that bug fix is released, the previous two lines should be changed to:
        # self.quaternion = R.rotation.quaternion

    def __repr__(self):
        return "OrientationConstraint('{0}', {1}, {2}, {3})".format(self.link_name, self.quaternion, self.tolerances, self.weight)

    def copy(self):
        cls = type(self)
        return cls(self.link_name, self.quaternion[:], self.tolerances[:], self.weight)


class PositionConstraint(Constraint):
    """Constrains a link to be within a certain bounding volume.

    Attributes
    ----------
    link_name: string
        The name of the link this contraint refers to.
    bounding_volume: :class:`compas.geometry.BoundingVolume`
        The volume this constraint refers to.
    weight: float, optional
        A weighting factor for this constraint. Denotes relative importance to
        other constraints. Closer to zero means less important. Defaults to 1.

    Examples
    --------
    >>> from compas.geometry import Sphere
    >>> from compas_fab.robots import PositionConstraint
    >>> from compas_fab.robots import BoundingVolume
    >>> bv = BoundingVolume.from_sphere(Sphere((3,4,5), 0.5))
    >>> pc = PositionConstraint('link_0', bv, weight=1.)
    """

    def __init__(self, link_name, bounding_volume, weight=1.):
        super(PositionConstraint, self).__init__(self.POSITION, weight)
        self.link_name = link_name
        self.bounding_volume = bounding_volume
        self.weight = weight

    @classmethod
    def from_box(cls, link_name, box, weight=1.):
        """Creates a ``PositionConstraint`` from a :class:`compas.geometry.Box`.

        Examples
        --------
        >>> from compas.geometry import Frame
        >>> from compas.geometry import Box
        >>> box = Box(Frame.worldXY(), 4, 4, 4)
        >>> pc = PositionConstraint.from_box('link_0', box)
        """
        bounding_volume = BoundingVolume.from_box(box)
        return cls(link_name, bounding_volume, weight)

    @classmethod
    def from_sphere(cls, link_name, sphere, weight=1.):
        """Creates a ``PositionConstraint`` from a :class:`compas.geometry.Sphere`.

        Examples
        --------
        >>> from compas_fab.robots import PositionConstraint
        >>> from compas.geometry import Sphere
        >>> sphere = Sphere((3,4,5), 0.5)
        >>> pc = PositionConstraint.from_sphere('link_0', sphere, weight=1.)
        """
        bounding_volume = BoundingVolume.from_sphere(sphere)
        return cls(link_name, bounding_volume, weight)

    @classmethod
    def from_mesh(cls, link_name, mesh, weight=1.):
        """Creates a ``PositionConstraint`` from a :class:`compas.datastructures.Mesh`.

        Examples
        --------
        >>> from compas.datastructures import Mesh
        >>> import compas
        >>> mesh = Mesh.from_obj(compas.get('faces.obj'))
        >>> pc = PositionConstraint.from_mesh('link_0', mesh)
        """
        bounding_volume = BoundingVolume.from_mesh(mesh)
        return cls(link_name, bounding_volume, weight)

    def scale(self, scale_factor):
        self.bounding_volume.scale(scale_factor)

    def transform(self, transformation):
        self.bounding_volume.transform(transformation)

    def __repr__(self):
        return "PositionConstraint('{0}', {1}, {2})".format(self.link_name, self.bounding_volume, self.weight)

    def copy(self):
        cls = type(self)
        return cls(self.link_name, self.bounding_volume.copy(), self.weight)
