from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.data import Data
from compas.geometry import Rotation
from compas.geometry import Scale
from compas.geometry import Sphere

from compas_fab.utilities import from_tcf_to_t0cf

__all__ = ["BoundingVolume", "Constraint", "JointConstraint", "OrientationConstraint", "PositionConstraint"]


class BoundingVolume(Data):
    """A container for describing a bounding volume.

    Parameters
    ----------
    volume_type
        The type of bounding volume, one of :attr:`BoundingVolume.VOLUME_TYPES`.
    volume : :class:`compas.datastructures.Mesh` or :class:`compas.geometry.Primitive`
        The volume can be either a :class:`compas.geometry.Box`, a
        :class:`compas.geometry.Sphere`, or a
        :class:`compas.datastructures.Mesh`.

    Attributes
    ----------
    volume_type
        The type of bounding volume, one of :attr:`BoundingVolume.VOLUME_TYPES`.
    volume : :class:`compas.datastructures.Mesh` or :class:`compas.geometry.Primitive`
        The volume can be either a :class:`compas.geometry.Box`, a
        :class:`compas.geometry.Sphere`, or a
        :class:`compas.datastructures.Mesh`.

    Notes
    -----
    `BoundingVolume.BOX`
        Box bounding volume type.
    `BoundingVolume.SPHERE`
        Sphere bounding volume type.
    `BoundingVolume.MESH`
        Mesh bounding volume type.
    `BoundingVolume.VOLUME_TYPES`
        List of supported bounding volume types.
    """

    #: Box bounding volume type
    BOX = 1
    #: Sphere bounding volume type
    SPHERE = 2
    #: Mesh bounding volume type
    MESH = 3

    #: List of supported volume types
    VOLUME_TYPES = (BOX, SPHERE, MESH)

    def __init__(self, volume_type, volume):
        if volume_type not in self.VOLUME_TYPES:
            raise ValueError("Type must be one of {}".format(self.VOLUME_TYPES))
        self.type = volume_type
        self.volume = volume

    def __data__(self):
        return {
            "volume_type": self.type,
            "volume": self.volume,
        }

    @classmethod
    def from_box(cls, box):
        """Create a :class:`BoundingVolume` from a :class:`compas.geometry.Box`.

        Parameters
        ----------
        box : :class:`compas.geometry.Box`
            Box to define :class:`BoundingVolume` with.

        Returns
        -------
        :class:`BoundingVolume`

        Examples
        --------
        >>> from compas.geometry import Frame
        >>> from compas.geometry import Box
        >>> from compas_fab.robots import BoundingVolume
        >>> box = Box(1., 1., 1.)
        >>> bv = BoundingVolume.from_box(box)
        >>> bv.type
        1
        """
        return cls(cls.BOX, box)

    @classmethod
    def from_sphere(cls, sphere):
        """Create a :class:`BoundingVolume` from a :class:`compas.geometry.Sphere`.

        Parameters
        ----------
        sphere : :class:`compas.geometry.Sphere`
            Sphere to define :class:`BoundingVolume` with.

        Returns
        -------
        :class:`BoundingVolume`

        Examples
        --------
        >>> from compas.geometry import Sphere
        >>> from compas_fab.robots import BoundingVolume
        >>> sphere = Sphere(5.0, Frame.worldXY())
        >>> bv = BoundingVolume.from_sphere(sphere)
        >>> bv.type
        2
        """
        return cls(cls.SPHERE, sphere)

    @classmethod
    def from_mesh(cls, mesh):
        """Create a :class:`BoundingVolume` from a :class:`compas.datastructures.Mesh`.

        Parameters
        ----------
        mesh : :class:`compas.datastructures.Mesh`

        Returns
        -------
        :class:`BoundingVolume`
            Mesh to define :class:`BoundingVolume` with.

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
        """Scale the volume uniformly.

        Parameters
        ----------
        scale_factor : :obj:`float`
            Scale factor to use in scaling operation.
        """
        S = Scale.from_factors([scale_factor] * 3)
        self.transform(S)

    def transform(self, transformation):
        """Transform the volume using a :class:`compas.geometry.Transformation`.

        Parameters
        ----------
        transformation : :class:`compas.geometry.Transformation`
            The transformation to apply on the :class:`BoundingVolume`.
        """
        self.volume.transform(transformation)

    def __repr__(self):
        """Printable representation of :class:`BoundingVolume`."""
        return "BoundingVolume({!r}, {!r})".format(self.type, self.volume)

    def copy(self):
        """Make a copy of this :class:`BoundingVolume`.

        Returns
        -------
        :class:`BoundingVolume`
            A copy.
        """
        cls = type(self)
        return cls(self.type, self.volume.copy())


class Constraint(Data):
    """Base class for robot constraints.

    Parameters
    ----------
    constraint_type
        Constraint type, one of :attr:`Constraint.CONSTRAINT_TYPES`.
    weight : :obj:`float`, optional
        A weighting factor for this constraint. Denotes relative importance to
        other constraints. Closer to zero means less important. Defaults to
        ``1``.

    Attributes
    ----------
    constraint_type
        Constraint type, one of :attr:`Constraint.CONSTRAINT_TYPES`.
    weight : :obj:`float`
        A weighting factor for this constraint. Denotes relative importance to
        other constraints. Closer to zero means less important.

    Notes
    -----
    Constraint.JOINT
        Joint constraint type.
    Constraint.POSITION
        Positional constraint type.
    Constraint.ORIENTATION
        Orientational constraint type.
    Constraint.CONSTRAINT_TYPES
        List of possible constraint types.
    """

    #: Joint constraint type.
    JOINT = 1
    #: Positional constraint type.
    POSITION = 2
    #: Orientational constraint type.
    ORIENTATION = 3

    #:  List of possible constraint types.
    CONSTRAINT_TYPES = (JOINT, POSITION, ORIENTATION)

    def __init__(self, constraint_type, weight=1.0):
        if constraint_type not in self.CONSTRAINT_TYPES:
            raise ValueError("Type must be %d, %d or %d" % self.CONSTRAINT_TYPES)
        self.type = constraint_type
        self.weight = weight

    def __data__(self):
        return {
            "constraint_type": self.type,
            "weight": self.weight,
        }

    def transform(self, transformation):
        """Transform the :class:`Constraint`."""
        pass

    def scale(self, scale_factor):
        """Scale the :class:`Constraint`."""
        pass

    def scaled(self, scale_factor):
        """Get a scaled copy of this :class:`Constraint`.

        Parameters
        ----------
        scale_factor : :obj:`float`
            Scale factor used to scale the :class:`Constraint`.
        """
        c = self.copy()
        c.scale(scale_factor)
        return c

    def copy(self):
        """Create a copy of this :class:`Constraint`.

        Returns
        -------
        :class:`BoundingVolume`
        """
        cls = type(self)
        return cls(self.type, self.weight)


class JointConstraint(Constraint):
    """Constrains the value of a joint to be within a certain bound.

    Parameters
    ----------
    joint_name : :obj:`str`
        The name of the joint this contraint refers to.
    value : :obj:`float`
        The targeted value for that joint.
    tolerance_above : :obj:`float`
        Tolerance above the targeted joint value, in radians. Defaults to ``0``.
    tolerance_below : :obj:`float`
        Tolerance below the targeted joint value, in radians. Defaults to ``0``.
    weight : :obj:`float`, optional
        A weighting factor for this constraint. Denotes relative importance to
        other constraints. Closer to zero means less important. Defaults to
        ``1``.

    Attributes
    ----------
    joint_name : :obj:`str`
        The name of the joint this contraint refers to.
    value : :obj:`float`
        The targeted value for that joint.
    tolerance_above : :obj:`float`
        Tolerance above the targeted joint value, in radians.
    tolerance_below : :obj:`float`
        Tolerance below the targeted joint value, in radians.
    weight : :obj:`float`
        A weighting factor for this constraint. Denotes relative importance to
        other constraints. Closer to zero means less important.

    Examples
    --------
    >>> from compas_fab.robots import JointConstraint
    >>> jc = JointConstraint("joint_0", 1.4, 0.1, 0.1, 1.0)
    """

    def __init__(self, joint_name, value, tolerance_above=0.0, tolerance_below=0.0, weight=1.0):
        super(JointConstraint, self).__init__(self.JOINT, weight)
        self.joint_name = joint_name
        self.value = value
        self.tolerance_above = abs(tolerance_above)
        self.tolerance_below = abs(tolerance_below)

    def __data__(self):
        return {
            "joint_name": self.joint_name,
            "value": self.value,
            "tolerance_above": self.tolerance_above,
            "tolerance_below": self.tolerance_below,
            "weight": self.weight,
        }

    def scale(self, scale_factor):
        """Scale (multiply) the constraint with a factor.

        Parameters
        ----------
        scale_factor : :obj:`float`
            Factor used to multiply the joint value and tolerance bounds with.
        """
        self.value *= scale_factor
        self.tolerance_above *= scale_factor
        self.tolerance_below *= scale_factor

    def __repr__(self):
        """Printable representation of :class:`JointConstraint`."""
        return "JointConstraint({!r}, {!r}, {!r}, {!r}, {!r})".format(
            self.joint_name, self.value, self.tolerance_above, self.tolerance_below, self.weight
        )

    def copy(self):
        """Create a copy of this :class:`JointConstraint`.

        Returns
        -------
        :class:`JointConstraint`
        """
        cls = type(self)
        return cls(self.joint_name, self.value, self.tolerance_above, self.tolerance_below, self.weight)

    @classmethod
    def joint_constraints_from_configuration(cls, configuration, tolerances_above, tolerances_below):
        """Create joint constraints for all joints of the configuration.
        One constraint is created for each joint.

        Parameters
        ----------
        configuration: :class:`Configuration`
            The target configuration.
        tolerances_above: :obj:`list` of :obj:`float`
            The tolerances above the targeted configuration's joint value on
            each of the joints, defining the upper bound in radians to be
            achieved. If only one value is passed in the list, it will be used to create
            upper bounds for all joint constraints.
        tolerances_below: :obj:`list` of :obj:`float`
            The tolerances below the targeted configuration's joint value on
            each of the joints, defining the upper bound in radians to be
            achieved. If only one value is passed, it will be used to create
            lower bounds for all joint constraints.

        Returns
        -------
        :obj:`list` of :class:`JointConstraint`

        Raises
        ------
        :exc:`ValueError`
            If the passed configuration does not have joint names.
        :exc:`ValueError`
            If the passed list of tolerance values have a different length than
            the configuration.

        Notes
        -----
        Make sure that you are using the correct tolerance units if your robot
        has different joint types defined.

        """
        joint_names = configuration.joint_names
        joint_values = configuration.joint_values
        if not joint_names:
            raise ValueError("The passed configuration has no joint_names")

        if len(joint_names) != len(configuration.joint_values):
            raise ValueError(
                "The passed configuration has {} joint_names but {} joint_values".format(
                    len(joint_names), len(joint_values)
                )
            )
        if len(tolerances_above) == 1:
            tolerances_above = tolerances_above * len(joint_names)
        elif len(tolerances_above) != len(configuration.joint_values):
            raise ValueError(
                "The number of `tolerances_above` values should either be 1 or equal to the number of joint_values ({}), current number of `tolerances_above` values: {}".format(
                    len(configuration.joint_values), len(tolerances_above)
                )
            )
        if len(tolerances_below) == 1:
            tolerances_below = tolerances_below * len(joint_names)
        elif len(tolerances_below) != len(configuration.joint_values):
            raise ValueError(
                "The number of `tolerances_above` values should either be 1 or equal to the number of joint_values ({}), current number of `tolerances_above` values: {}".format(
                    len(configuration.joint_values), len(tolerances_below)
                )
            )

        constraints = []
        for name, value, tolerance_above, tolerance_below in zip(
            joint_names, configuration.joint_values, tolerances_above, tolerances_below
        ):
            constraints.append(JointConstraint(name, value, tolerance_above, tolerance_below))
        return constraints


class OrientationConstraint(Constraint):
    """Constrains a link to be within a certain orientation.

    Parameters
    ----------
    link_name : :obj:`str`
        The name of the link this contraint refers to.
    quaternion : :obj:`list` of :obj:`float`
        The desired orientation of the link specified by a quaternion in the
        order of ``[w, x, y, z]``.
    tolerances : :obj:`list` of :obj:`float`, optional
        Error tolerances :math:`t_{i}` for each of the frame's axes. If only one
        value is passed it will be used for all 3 axes. The respective bound to
        be achieved is :math:`(a_{i} - t_{i}, a_{i} + t_{i})`. Defaults to
        ``[0.01, 0.01, 0.01]``.
    weight : :obj:`float`, optional
        A weighting factor for this constraint. Denotes relative importance to
        other constraints. Closer to zero means less important. Defaults to
        ``1``.

    Attributes
    ----------
    link_name : :obj:`str`
        The name of the link this contraint refers to.
    quaternion : :obj:`list` of :obj:`float`
        The desired orientation of the link specified by a quaternion in the
        order of ``[w, x, y, z]``.
    tolerances : :obj:`list` of :obj:`float`
        Error tolerances :math:`t_{i}` for each of the frame's axes. If only one
        value is passed it will be used for all 3 axes. The respective bound to
        be achieved is :math:`(a_{i} - t_{i}, a_{i} + t_{i})`.
    weight : :obj:`float`
        A weighting factor for this constraint. Denotes relative importance to
        other constraints. Closer to zero means less important.

    Notes
    -----
    The rotation tolerance for an axis is defined by the other vector component
    values for rotation around corresponding axis.

    If you specify the tolerance vector with ``[0.01, 0.01, 6.3]``, it means
    that the frame's x-axis and y-axis are allowed to rotate about the z-axis
    by an angle of 6.3 radians, whereas the z-axis can only change 0.01.

    Examples
    --------
    >>> from compas.geometry import Frame
    >>> from compas_fab.robots import OrientationConstraint
    >>> frame = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
    >>> oc = OrientationConstraint("link_0", frame.quaternion)
    """

    def __init__(self, link_name, quaternion, tolerances=None, weight=1.0):
        super(OrientationConstraint, self).__init__(self.ORIENTATION, weight)
        self.link_name = link_name
        self.quaternion = [float(a) for a in list(quaternion)]
        self.tolerances = [float(a) for a in list(tolerances)] if tolerances else [0.01] * 3

    def __data__(self):
        return {
            "link_name": self.link_name,
            "quaternion": self.quaternion,
            "tolerances": self.tolerances,
            "weight": self.weight,
        }

    def transform(self, transformation):
        """Transform the volume using a :class:`compas.geometry.Transformation`.

        Parameters
        ----------
        transformation : :class:`compas.geometry.Transformation`
        """
        R = Rotation.from_quaternion(self.quaternion)
        R = transformation * R

        self.quaternion = R.rotation.quaternion

    def __repr__(self):
        """Printable representation of :class:`OrientationConstraint`."""
        return "OrientationConstraint({!r}, {!r}, {!r}, {!r})".format(
            self.link_name, self.quaternion, self.tolerances, self.weight
        )

    def copy(self):
        """Create a copy of this :class:`OrientationConstraint`.

        Returns
        -------
        :class:`OrientationConstraint`
        """
        cls = type(self)
        return cls(self.link_name, self.quaternion[:], self.tolerances[:], self.weight)

    @classmethod
    def from_frame(cls, frame_WCF, tolerances_orientation, link_name, tool_coordinate_frame=None, weight=1.0):
        """Create an :class:`OrientationConstraint` from a frame on the group's end-effector link.
        Only the orientation of the frame is considered for the constraint, expressed
        as a quaternion.

        Parameters
        ----------
        frame_WCF: :class:`compas.geometry.Frame`
            The frame from which we create the orientation constraint.
        tolerances_orientation: :obj:`list` of :obj:`float`
            Error tolerances :math:`t_{i}` for each of the frame's axes in
            radians. If only one value is passed in the list it will be uses for all 3 axes.
        link_name : :obj:`str`
            The name of the end-effector link. Necessary for creating the position constraint.
        tool_coordinate_frame : :class:`compas.geometry.Frame`, optional
            The tool tip coordinate frame relative to the flange of the robot.
            If not specified, the target frame is relative to the robot's flange.
        weight : :obj:`float`, optional
            A weighting factor for this constraint. Denotes relative importance to
            other constraints. Closer to zero means less important. Defaults to
            ``1``.

        Returns
        -------
        :class:`OrientationConstraint`

        Raises
        ------
        :exc:`ValueError`
            If tolerance axes given are not one or three values.

        Notes
        -----
        The rotation tolerance for an axis is defined by the other vector
        component values for rotation around corresponding axis.
        If you specify the tolerances_orientation vector with ``[0.01, 0.01, 6.3]``, it
        means that the frame's x-axis and y-axis are allowed to rotate about the
        z-axis by an angle of 6.3 radians, whereas the z-axis would only rotate
        by 0.01.


        Examples
        --------
        >>> from compas_fab.robots.ur5 import Robot
        >>> robot = Robot()
        >>> frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
        >>> tolerances_orientation = [math.radians(1)] * 3
        >>> group = robot.main_group_name
        >>> ee_link_name = robot.get_end_effector_link_name(group)
        >>> OrientationConstraint.from_frame(frame, tolerances_orientation, ee_link_name)
        OrientationConstraint('ee_link', [0.5, 0.5, 0.5, 0.5], [0.017453292519943295, 0.017453292519943295, 0.017453292519943295], 1.0)
        """

        if tool_coordinate_frame:
            frame_WCF = from_tcf_to_t0cf(frame_WCF, tool_coordinate_frame)

        tolerances_orientation = list(tolerances_orientation)
        if len(tolerances_orientation) == 1:
            tolerances_orientation *= 3
        elif len(tolerances_orientation) != 3:
            raise ValueError("`tolerances_orientation` must be a list with either 1 or 3 values")

        return cls(link_name, frame_WCF.quaternion, tolerances_orientation, weight)


class PositionConstraint(Constraint):
    """Constrains a link to be within a certain bounding volume.

    Parameters
    ----------
    link_name : :obj:`str`
        The name of the link this contraint refers to.
    bounding_volume : :class:`BoundingVolume`
        The volume this constraint refers to.
    weight : :obj:`float`, optional
        A weighting factor for this constraint. Denotes relative importance to
        other constraints. Closer to zero means less important. Defaults to
        ``1``.

    Attributes
    ----------
    link_name : :obj:`str`
        The name of the link this contraint refers to.
    bounding_volume : :class:`BoundingVolume`
        The volume this constraint refers to.
    weight : :obj:`float`
        A weighting factor for this constraint. Denotes relative importance to
        other constraints. Closer to zero means less important.

    Examples
    --------
    >>> from compas.geometry import Sphere
    >>> from compas_fab.robots import PositionConstraint
    >>> from compas_fab.robots import BoundingVolume
    >>> bv = BoundingVolume.from_sphere(Sphere(0.5, point=[3,4,5]))
    >>> pc = PositionConstraint('link_0', bv, weight=1.)
    """

    def __init__(self, link_name, bounding_volume, weight=1.0):
        super(PositionConstraint, self).__init__(self.POSITION, weight)
        self.link_name = link_name
        self.bounding_volume = bounding_volume
        self.weight = weight

    def __data__(self):
        return {
            "link_name": self.link_name,
            "bounding_volume": self.bounding_volume,
            "weight": self.weight,
        }

    @classmethod
    def from_frame(cls, frame_WCF, tolerance_position, link_name, tool_coordinate_frame=None, weight=1.0):
        """Create a :class:`PositionConstraint` from a frame on the group's end-effector link.
        Only the position of the frame is considered for the constraint.

        Parameters
        ----------
        frame_WCF : :class:`compas.geometry.Frame`
            The frame from which we create position and orientation constraints.
        tolerance_position : :obj:`float`
            The allowed tolerance to the frame's position (defined in the
            robot's units).
        link_name : :obj:`str`
            The name of the end-effector link. Necessary for creating the position constraint.
        tool_coordinate_frame : :class:`compas.geometry.Frame`, optional
            The tool tip coordinate frame relative to the flange of the robot.
            If not specified, the target frame is relative to the robot's flange.
        weight : :obj:`float`
            A weighting factor for this constraint. Denotes relative importance to
            other constraints. Closer to zero means less important. Defaults to ``1``.

        Returns
        -------
        :class:`PositionConstraint`

        See Also
        --------
        :meth:`PositionConstraint.from_box`
        :meth:`PositionConstraint.from_mesh`
        :meth:`PositionConstraint.from_sphere`

        Examples
        --------
        >>> from compas_fab.robots.ur5 import Robot
        >>> robot = Robot()
        >>> frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
        >>> tolerance_position = 0.001
        >>> group = robot.main_group_name
        >>> ee_link_name = robot.get_end_effector_link_name(group)
        >>> PositionConstraint.from_frame(frame, tolerance_position, ee_link_name)                                 # doctest: +SKIP
        PositionConstraint('ee_link', BoundingVolume(2, Sphere(Point(0.400, 0.300, 0.400), 0.001)), 1.0)    # doctest: +SKIP
        """

        if tool_coordinate_frame:
            frame_WCF = from_tcf_to_t0cf(frame_WCF, tool_coordinate_frame)

        sphere = Sphere(radius=tolerance_position, point=frame_WCF.point)
        return cls.from_sphere(link_name, sphere, weight)

    @classmethod
    def from_box(cls, link_name, box, weight=1.0):
        """Create a :class:`PositionConstraint` from a :class:`compas.geometry.Box`.

        Parameters
        ----------
        link_name: :obj:`str`
            The name of the link this contraint refers to.
        box : :class:`compas.geometry.Box`
            Box defining the bounding volume this constraint refers to.
        weight : :obj:`float`, optional
            A weighting factor for this constraint. Denotes relative importance to
            other constraints. Closer to zero means less important. Defaults to ``1``.

        Returns
        -------
        :class:`PositionConstraint`

        Examples
        --------
        >>> from compas.geometry import Frame
        >>> from compas.geometry import Box
        >>> box = Box(4, 4, 4, Frame.worldXY())
        >>> pc = PositionConstraint.from_box('link_0', box)
        """
        bounding_volume = BoundingVolume.from_box(box)
        return cls(link_name, bounding_volume, weight)

    @classmethod
    def from_sphere(cls, link_name, sphere, weight=1.0):
        """Create a :class:`PositionConstraint` from a :class:`compas.geometry.Sphere`.

        Parameters
        ----------
        link_name : :obj:`str`
            The name of the link this contraint refers to.
        sphere : :class:`compas.geometry.Sphere`
            Sphere defining the bounding volume this constraint refers to.
        weight : :obj:`float`
            A weighting factor for this constraint. Denotes relative importance to
            other constraints. Closer to zero means less important. Defaults to ``1``.

        Returns
        -------
        :class:`PositionConstraint`

        Examples
        --------
        >>> from compas_fab.robots import PositionConstraint
        >>> from compas.geometry import Sphere
        >>> sphere = Sphere(radius=0.5, point=[3,4,5])
        >>> pc = PositionConstraint.from_sphere('link_0', sphere, weight=1.)
        """
        bounding_volume = BoundingVolume.from_sphere(sphere)
        return cls(link_name, bounding_volume, weight)

    @classmethod
    def from_point(cls, link_name, point, tolerance_position, weight=1.0):
        """Create a :class:`PositionConstraint` from a point.

        Parameters
        ----------
        link_name : :obj:`str`
            The name of the link this contraint refers to.
        point : :class:`compas.geometry.Point`
            Point defining the bounding volume this constraint refers to.
        tolerance_position : :obj:`float`
            The allowed tolerance to the point's position (defined in the
            robot's units).
        weight : :obj:`float`
            A weighting factor for this constraint. Denotes relative importance to
            other constraints. Closer to zero means less important. Defaults to ``1``.

        Returns
        -------
        :class:`PositionConstraint`

        """
        sphere = Sphere(radius=tolerance_position, point=point)
        return cls.from_sphere(link_name, sphere, weight)

    @classmethod
    def from_mesh(cls, link_name, mesh, weight=1.0):
        """Create a :class:`PositionConstraint` from a :class:`compas.datastructures.Mesh`.

        Parameters
        ----------
        link_name : :obj:`str`
            The name of the link this contraint refers to.
        mesh : :class:`compas.datastructures.Mesh`
            Mesh defining the bounding volume this constraint refers to.
        weight : :obj:`float`
            A weighting factor for this constraint. Denotes relative importance
            to other constraints. Closer to zero means less important. Defaults
            to ``1``.

        Returns
        -------
        :class:`PositionConstraint`

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
        """Scale the :attr:`bounding_volume` uniformely.

        Parameters
        ----------
        scale_factor : :obj:`float`
            Factor to scale constraining :attr:`bounding_volume`.
        """
        self.bounding_volume.scale(scale_factor)

    def transform(self, transformation):
        """Transform the :attr:`bounding_volume` using a :class:`compas.geometry.Transformation`.

        Parameters
        ----------
        transformation : :class:`compas.geometry.Transformation`
        """
        self.bounding_volume.transform(transformation)

    def __repr__(self):
        """Printable representation of :class:`PositionConstraint`."""
        return "PositionConstraint({!r}, {!r}, {!r})".format(self.link_name, self.bounding_volume, self.weight)

    def copy(self):
        """Create a copy of this :class:`PositionConstraint`.

        Returns
        -------
        :class:`PositionConstraint`
        """
        cls = type(self)
        return cls(self.link_name, self.bounding_volume.copy(), self.weight)
