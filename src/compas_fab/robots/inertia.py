from compas.geometry import Point


class Inertia():
    """The moments of inertia represent the spatial distribution of mass in a rigid body.

    It depends on the mass, size, and shape of a rigid body with units of
    [mass * m**2]. The moments of inertia can be expressed as the components of
    a symmetric positive-definite 3x3 matrix, with 3 diagonal elements, and 3
    unique off-diagonal elements. Each inertia matrix is defined relative to a
    coordinate frame or set of axes.

    Attributes
    ----------
    inertia_tensor : list of float
        A symmetric positive-definite 3x3 matrix:
        | ixx ixy ixz |
        | ixy iyy iyz |
        | ixz iyz izz |
        with [ixx, iyy, izz] as the principal moments of inertia and
        [ixy, ixz, iyz] as the products of inertia.
    mass: float
        The mass of the object in kg.
    center_of_mass : :class:`Point`
        The center of mass of the object in meters.

    Examples
    --------
    >>> inertia = Inertia([[0] * 3] * 3, 1., Point(0.1, 3.1, 4.4))
    >>> inertia
    Inertia([[0, 0, 0], [0, 0, 0], [0, 0, 0]], 1.0, Point(0.100, 3.100, 4.400))
    >>> inertia.principal_moments
    [0, 0, 0]

    Notes
    -----
    Assuming uniform mass density, inertial data can be obtained using the
    free software MeshLab, refering to this great tutorial:
    http://gazebosim.org/tutorials?tut=inertia
    """

    def __init__(self, inertia_tensor, mass, center_of_mass):
        self.inertia_tensor = inertia_tensor
        self.mass = mass
        self.center_of_mass = Point(*center_of_mass)

    @property
    def principal_moments(self):
        """Returns the diagonal elements of the inertia tensor [ixx, iyy, izz]
        """
        inertia_tensor = self.inertia_tensor
        return [inertia_tensor[0][0], inertia_tensor[1][1], inertia_tensor[2][2]]

    def __repr__(self):
        return "Inertia({0}, {1}, {2})".format(self.inertia_tensor, self.mass, self.center_of_mass)

    @staticmethod
    def calculate_inertia_tensor(cls, mesh):
        """Returns the inertia tensor.
        """
        raise NotImplementedError
