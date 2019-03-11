from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

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
        """
        return cls(cls.BOX, box)

    @classmethod
    def from_sphere(cls, sphere):
        """Creates a `BoundingVolume` from a :class:`compas.geometry.Sphere`.

        Parameters
        ----------
        sphere: `compas.geometry.Sphere`

        Examples
        --------
        >>> from compas.geometry import Sphere
        >>> from compas_fab.robots import BoundingVolume
        >>> sphere = Sphere((1., 1., 1.), 5.)
        >>> bv = BoundingVolume.from_sphere(sphere)
        """
        return cls(cls.SPHERE, sphere)

    @classmethod
    def from_mesh(cls, mesh):
        """Creates a `BoundingVolume` from a :class:`compas.datastructures.Mesh`.

        Parameters
        ----------
        sphere: `compas.datastructures.Mesh`

        Examples
        --------
        >>> import compas
        >>> from compas.datastructures import Mesh
        >>> from compas_fab.robots import BoundingVolume
        >>> mesh = Mesh.from_obj(compas.get('faces.obj'))
        >>> bv = BoundingVolume.from_mesh(Mesh)
        """
        return cls(cls.MESH, mesh)


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
    possible_types = [JOINT, POSITION, ORIENTATION]

    def __init__(self, type, weight=1.):
        if type not in self.possible_types:
            raise ValueError("Type must be %d, %d or %d" % tuple(self.possible_types))
        self.type = type
        self.weight = weight


class JointConstraint(Constraint):
    """Constrains the value of a joint to be within a certain bound.

    Attributes
    ----------
    joint_name: string
        The name of the joint this contraint refers to.
    value: float
        The targeted value for that joint.
    tolerance: float
        The bound to be achieved is [value - tolerance, value + tolerance]
    weight: float, optional
        A weighting factor for this constraint. Denotes relative importance to
        other constraints. Closer to zero means less important. Defaults to 1.

    Examples
    --------
    >>> from compas_fab.robots import JointConstraint
    >>> jc = JointConstraint("joint_0", 1.4, 0.1)

    """
    def __init__(self, joint_name, value, tolerance=0., weight=1.):
        super(JointConstraint, self).__init__(self.JOINT, weight)
        self.joint_name = joint_name
        self.value = value
        self.tolerance = tolerance


class OrientationConstraint(Constraint):
    """Constrains a link to be within a certain orientation.

    Attributes
    ----------
    link_name: string
        The name of the link this contraint refers to.
    euler_angles: list of float
        The desired orientation of the link specified by euler angles,
        describing rotations about static 'xyz' axes.
    tolerances: list of float, optional
        Error tolerances ti for each of the euler angles ai. The respective
        bound to be achieved is [ai - ti, ai + ti]. Defaults to [0.,0.,0.].
    weight: float, optional
        A weighting factor for this constraint. Denotes relative importance to
        other constraints. Closer to zero means less important. Defaults to 1.

    Examples
    --------
    >>> from compas.geometry import Frame
    >>> from compas_fab.robots import OrientationConstraint
    >>> frame = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
    >>> oc = OrientationConstraint("link_0", frame.euler_angles(), tolerances=[0,0,0])

    """
    def __init__(self, link_name, euler_angles, tolerances=None, weight=1.):
        super(OrientationConstraint, self).__init__(self.ORIENTATION, weight)
        self.link_name = link_name
        self.euler_angles = [float(a) for a in list(euler_angles)]
        self.tolerances = [float(a) for a in list(tolerances)] if tolerances else [0., 0., 0.]


class PositionConstraint(Constraint):
    """Constrains a link to be within a certain position or volume.

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
        """Creates a `PositionConstraint` from a :class:`compas.geometry.Box`.

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
        """Creates a `PositionConstraint` from a :class:`compas.geometry.Sphere`.

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
        """Creates a `PositionConstraint` from a :class:`compas.datastructures.Mesh`.

        Examples
        --------
        >>> from compas.datastructures import Mesh
        >>> import compas
        >>> mesh = Mesh.from_obj(compas.get('faces.obj'))
        >>> pc = PositionConstraint.from_mesh('link_0', mesh)
        """
        bounding_volume = BoundingVolume.from_mesh(mesh)
        return cls(link_name, bounding_volume, weight)


if __name__ == "__main__":

    import doctest
    doctest.testmod()
    #print([name for name in dir() if not name.startswith('_')])
