"""
This library for transformations partly derived and was re-implemented from the
following online resources:

    * http://www.lfd.uci.edu/~gohlke/code/transformations.py.html
    * http://www.euclideanspace.com/maths/geometry/rotations/
    * http://code.activestate.com/recipes/578108-determinant-of-matrix-of-any-order/
    * http://blog.acipo.com/matrix-inversion-in-javascript/

Many thanks to Christoph Gohlke, Martin John Baker, Sachin Joglekar and Andrew
Ippoliti for providing code and documentation.
"""

import math
from copy import deepcopy

# Support pre-release versions of compas
try:
    from compas.geometry.basic import multiply_matrix_vector
    from compas.geometry.basic import multiply_matrices
except ImportError:
    from compas.geometry.utilities import multiply_matrix_vector
    from compas.geometry.utilities import multiply_matrices

from compas.geometry.basic import dot_vectors
from compas.geometry.basic import normalize_vector
from compas.geometry.basic import cross_vectors
from compas.geometry.basic import length_vector
from compas.geometry.basic import subtract_vectors
from compas.geometry.basic import scale_vector
from compas.geometry.basic import norm_vector
from compas.geometry.basic import transpose_matrix

from compas.geometry.transformations import homogenize
from compas.geometry.transformations import dehomogenize
from compas.geometry.transformations import scale_matrix
from compas.geometry.transformations import translation_matrix

from compas_fab.fab.utilities.numbers import allclose

__author__ = ['Romana Rust <rust@arch.ethz.ch>', ]


# epsilon for testing whether a number is close to zero
_EPS = 1e-16

# TODO: need to be moved ideally to compas.geometry.basic


def determinant(M, check=True):
    """Calculates the determinant of a square matrix M.

    Args:
        M (:obj:`iterable` of :obj:`iterable` of :obj:`float`): The square
            matrix of any dimension.

    Raises:
        ValueError: If matrix is not a square matrix.

    Returns:
        (float): The determinant.
    """

    dim = len(M)

    if check:
        for c in M:
            if len(c) != dim:
                raise ValueError("Not a square matrix")

    if (dim == 2):
        return M[0][0] * M[1][1] - M[0][1] * M[1][0]
    else:
        i = 1
        t = 0
        sum = 0
        for t in range(dim):
            d = {}
            for t1 in range(1, dim):
                m = 0
                d[t1] = []
                for m in range(dim):
                    if (m != t):
                        d[t1].append(M[t1][m])
            M1 = [d[x] for x in d]
            sum = sum + i * M[0][t] * determinant(M1, check=False)
            i = i * (-1)
        return sum


def inverse(M):
    """Calculates the inverse of a square matrix M.

    This method uses Gaussian elimination (elementary row operations) to
    calculate the inverse. The elementary row operations are a) swap 2 rows,
    b) multiply a row by a scalar, and c) add 2 rows.

    Args:
        M (:obj:`iterable` of :obj:`iterable` of :obj:`float`): The square
            matrix of any dimension.

    Raises:
        ValueError: If the matrix is not squared
        ValueError: If the matrix is singular.
        ValueError: If the matrix is not invertible.

    Returns:
        (:obj:`list` of :obj:`list` of :obj:`float`): The inverted matrix.
    """

    detM = determinant(M)  # raises ValueError if matrix is not squared

    if detM == 0:
        ValueError("The matrix is singular.")

    dim = len(M)

    # create identity I and copy M into C
    I = [[1. if i == j else 0. for i in range(4)] for j in range(4)]
    C = [[M[i][j] for i in range(4)] for j in range(4)]

    # Perform elementary row operations
    for i in range(dim):
        e = C[i][i]
        if e == 0:
            for ii in range(dim):
                if C[ii][i] != 0:
                    for j in range(dim):
                        e = C[i][j]
                        C[i][j] = C[ii][j]
                        C[ii][j] = e
                        e = I[i][j]
                        I[i][j] = I[ii][j]
                        I[ii][j] = e
                    break
            e = C[i][i]
            if e == 0:
                ValueError("Matrix not invertible")
        for j in range(dim):
            C[i][j] = C[i][j] / e
            I[i][j] = I[i][j] / e

        for ii in range(dim):
            if ii == i:
                continue
            e = C[ii][i]
            for j in range(dim):
                C[ii][j] -= e * C[i][j]
                I[ii][j] -= e * I[i][j]
    return I


def decompose(M):
    """Calculates the components of rotation, translation, scale, shear,
        and perspective of a given transformation matrix M.

    Returns:
        rotation (:obj:`list` of :obj:`float`): The rotation specified
            through the 3 Euler angles about static x, y, z axes TODO
        translation (:obj:`list` of :obj:`float`): The 3 values of
            translation
        scale (:obj:`list` of :obj:`float`): The 3 scale factors in x-, y-,
            and z-direction.
        shear (:obj:`list` of :obj:`float`): The 3 shear factors for x-y,
            x-z, and y-z axes. TODO make shear from these factors
        perspective (:obj:`list` of :obj:`float`): perspective partition
            of the matrix

    Raises:
        ValueError: If matrix is of wrong type or degenerative.

    Example:
    >>> T0 = translation_matrix([1, 2, 3])
    >>> scale, shear, angles, trans, persp = decompose_matrix(T0)
    >>> T1 = translation_matrix(trans)
    >>> numpy.allclose(T0, T1)
    True
    >>> S = scale_matrix(0.123)
    >>> scale, shear, angles, trans, persp = decompose_matrix(S)
    >>> scale[0]
    0.123
    >>> R0 = euler_matrix(1, 2, 3)
    >>> scale, shear, angles, trans, persp = decompose_matrix(R0)
    >>> R1 = euler_matrix(*angles)
    >>> numpy.allclose(R0, R1)
    True
    """

    detM = determinant(M)  # raises ValueError if matrix is not squared

    if detM == 0:
        ValueError("The matrix is singular.")

    # TODO: make a list conversion in transpose_matrix: map(list,
    Mt = map(list, transpose_matrix(M))

    if abs(Mt[3][3]) < _EPS:
        raise ValueError('The element [3,3] of the matrix is zero.')

    for i in range(4):
        for j in range(4):
            Mt[i][j] /= Mt[3][3]

    translation = [M[0][3], M[1][3], M[2][3]]

    # scale, shear, rotation
    # copy Mt[:3, :3] into row
    scale = [0.0, 0.0, 0.0]
    shear = [0.0, 0.0, 0.0]
    angles = [0.0, 0.0, 0.0]

    row = [[0, 0, 0] for i in range(3)]
    for i in range(3):
        for j in range(3):
            row[i][j] = Mt[i][j]

    scale[0] = norm_vector(row[0])
    for i in range(3):
        row[0][i] /= scale[0]
    shear[0] = dot_vectors(row[0], row[1])
    for i in range(3):
        row[1][i] -= row[0][i] * shear[0]
    scale[1] = norm_vector(row[1])
    for i in range(3):
        row[1][i] /= scale[1]
    shear[0] /= scale[1]
    shear[1] = dot_vectors(row[0], row[2])
    for i in range(3):
        row[2][i] -= row[0][i] * shear[1]
    shear[2] = dot_vectors(row[1], row[2])
    for i in range(3):
        row[2][i] -= row[0][i] * shear[2]
    scale[2] = norm_vector(row[2])
    for i in range(3):
        row[2][i] /= scale[2]
    shear[1] /= scale[2]
    shear[2] /= scale[2]

    if dot_vectors(row[0], cross_vectors(row[1], row[2])) < 0:
        scale = [-x for x in scale]
        row = [[-x for x in y] for y in row]

    # use base vectors??
    angles[1] = math.asin(-row[0][2])
    if math.cos(angles[1]):
        angles[0] = math.atan2(row[1][2], row[2][2])
        angles[2] = math.atan2(row[0][1], row[0][0])
    else:
        angles[0] = math.atan2(-row[2][1], row[1][1])
        angles[2] = 0.0

    # perspective
    if math.fabs(Mt[0][3]) > _EPS and math.fabs(Mt[1][3]) > _EPS and \
            math.fabs(Mt[2][3]) > _EPS:
        P = deepcopy(Mt)
        P[0][3], P[1][3], P[2][3], P[3][3] = 0.0, 0.0, 0.0, 1.0
        Ptinv = inverse(map(list, transpose_matrix(P)))
        perspective = multiply_matrix_vector(Ptinv, [Mt[0][3], Mt[1][3],
                                                     Mt[2][3], Mt[3][3]])
    else:
        perspective = [0.0, 0.0, 0.0, 1.0]

    return angles, translation, scale, shear, perspective


class Transformation(object):
    """The ``Transformation`` represents a 4x4 transformation matrix.

    It calculates transformation matrices for the operations of rotation,
    translation, reflection, scale, shear, and projection. The class contains
    methods for converting rotation matrices to axis-angle representations,
    Euler angles, and quaternions. It also allows to concatenate
    Transformations by multiplication, to calulate the inverse transformation
    and to decompose a transformation into its components of rotation,
    translation, scale, shear, and perspective. It follows the row-major order,
    such that translation components x, y, z are in the right column of the
    matrix, i.e. M[0][3], M[1][3], M[2][3] = x, y, z

    Examples:
        T = Transformation.from_matrix(M)
        T = Transformation.from_matrix(M)
        f = Frame()
        T.decompose()
        Tinv = T.inverse()

    """

    def __init__(self):
        self.matrix = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]]

    @classmethod
    def identity(cls):
        return cls()

    @classmethod
    def from_matrix(cls, matrix):
        """Creates a ``Transformation`` from a 4x4 two-dimensional list of
            numbers.

        Args:
            matrix (:obj:`iterable` of :obj:`iterable` of :obj:`float`)
        """
        T = cls()
        for i in range(4):
            for j in range(4):
                T.matrix[i][j] = float(matrix[i][j])
        return T

    @classmethod
    def from_list(cls, numbers):
        """Creates a ``Transformation`` from a list of 16 numbers.

        Since the transformation matrix follows the row-major order, the
        translational components must be at the list's indices 3, 7, 11.

        Args:
            numbers (:obj:`iterable` of :obj:`float`)

        Example:
            numbers = [1, 0, 0, 3, 0, 1, 0, 4, 0, 0, 1, 5, 0, 0, 0, 1]
            T = Transformation.from_list(numbers)
        """
        T = cls()
        for i in range(4):
            for j in range(4):
                T.matrix[i][j] = float(numbers[i * 4 + j])
        return T

    @classmethod
    def from_frame(cls, frame):
        """Computes a change of basis transformation from world XY to frame.

        It is the same as from_frame_to_frame(Frame.worldXY(), frame).

        Args:
            frame (:class:`Frame`): a frame describing the targeted Cartesian
                coordinate system

        Example:
            f = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
            T = Transformation.from_frame(f)
        """

        T = cls()
        T.matrix[0][0], T.matrix[1][0], T.matrix[2][0] = frame.xaxis
        T.matrix[0][1], T.matrix[1][1], T.matrix[2][1] = frame.yaxis
        T.matrix[0][2], T.matrix[1][2], T.matrix[2][2] = frame.zaxis
        T.matrix[0][3], T.matrix[1][3], T.matrix[2][3] = frame.point
        return T

    @classmethod
    def from_frame_to_frame(cls, frame_from, frame_to):
        """Computes a change of basis transformation between two frames.

        This transformation maps geometry from one Cartesian coordinate system
        defined by "frame_from" to the other Cartesian coordinate system
        defined by "frame_to".

        Args:
            frame_from (:class:`Frame`): a frame defining the original
                Cartesian coordinate system
            frame_to (:class:`Frame`): a frame defining the targeted
                Cartesian coordinate system

        Example:
            f1 = Frame([0, 0, 0], [0.12, 0.58, 0.81], [-0.80, 0.53, -0.26])
            f2 = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
            T = Transformation.from_frame_to_Frame(f1, f2)
        """
        T1 = Transformation.from_frame(frame_from)
        T2 = Transformation.from_frame(frame_to)
        return T2 * T1.inverse()

    def inverse(self):
        """Returns the inverse transformation.

        Example:
            f = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
            T = Transformation.from_frame(f)
            Tinv = T.inverse()
        """
        inv = inverse(self.matrix)
        return Transformation.from_matrix(inv)

    def decompose(self):
        """Calculates the transformation's components of rotation, translation,
            scale, shear, and perspective.

        Returns:

        Raises:
            ValueError: If matrix is of wrong type or degenerative.

        Example:

        """

        angles, translation, scale, shear, perspective = decompose(self.matrix)
        return angles, translation, scale, shear, perspective

    def rotation(self):
        """Get just rotation from transformation.

        TODO: decompose matrix ?

        Returns:
            (Rotation)
        """
        angles, translation, scale, shear, perspective = self.decompose()
        return Rotation.from_matrix(self.matrix)

    def translation(self):
        """Returns the 3 values of translation from the transformation.
        """
        return [self.matrix[0][3], self.matrix[1][3], self.matrix[2][3]]

    def shear(self):
        """
        """
        angles, translation, scale, shear, perspective = self.decompose()
        return shear

    def perspective(self):
        """
        """
        angles, translation, scale, shear, perspective = self.decompose()
        return shear

    def scale(self):
        """
        """
        angles, translation, scale, shear, perspective = self.decompose()
        return scale

    def transform_point(self, point):
        """Transforms a point.

        Args:
            point (:obj:`iterable of :obj:`float`)

        Example:
            f = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
            T = Transformation.from_frame(f)
            q = T.transform_point([0,0,0])
            print(allclose(f.origin, q))

        Returns:
            (:obj:`list of :obj:`float`): The transformed point.
        """

        ph = list(point) + [1.]  # make homogeneous coordinates
        pht = multiply_matrix_vector(self.matrix, ph)
        return pht[:3]

    def transform_points(self, points):
        """Transforms a list of points.

        Args:
            points (:obj:`iterable` of :obj:`iterable of :obj:`float`): A list
                of points.

        Example:
            f0 = Frame.worldXY()
            f1 = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
            T = Transformation.from_frame_to_frame(f0, f1)
            p1, p2, p3 = f1.point, add_vectors(f1.point, f1.xaxis), \
                add_vectors(f1.point, f1.yaxis)
            q1, q2, q3 = T.transform_points([f0.point, add_vectors(f0.point, \
                f0.xaxis), add_vectors(f0.point, f0.yaxis)])
            print(allclose(p1, q1))
            print(allclose(p2, q2))
            print(allclose(p3, q3))

        Returns:
            (:obj:`list` of :obj:`list of :obj:`float`): The transformed
                points.
        """
        P = homogenize(points)
        Pt = map(list, zip(*P))  # transpose matrix
        Pt_ = multiply_matrices(self.matrix, Pt)
        return map(list, zip(*Pt_[:3]))  # cutoff 1 and transpose again

    @property
    def list(self):
        """Flattens the transformation matrix into a list of numbers.
        """
        return [a for c in self.matrix for a in c]

    def concatenate(self, other):
        """Concatenate two transformations to one transformation.

        Args:
            other (:class:`Transformation`)

        Returns:
            (:class:`Transformation`)

        """
        return Transformation.from_matrix(multiply_matrices(self.matrix,
                                                            other.matrix))

    def __mul__(self, other):
        return self.concatenate(other)

    def __imul__(self, other):
        return self.concatenate(other)

    def __getitem__(self, key):
        i, j = key
        return self.matrix[i][j]

    def __setitem__(self, key, value):
        i, j = key
        self.matrix[i][j] = value

    def __iter__(self):
        return iter(self.matrix)

    def __eq__(self, other, tol=1e-05):
        try:
            M = self.matrix
            O = other.matrix
            for i in range(4):
                for j in range(4):
                    if math.fabs(M[i][j] - O[i][j]) > tol:
                        return False
            return True
        except BaseException:
            raise TypeError("Wrong input type.")

    def __repr__(self):
        s = "[[%s],\n" % ",".join([("%.4f" % n).rjust(10)
                                   for n in self.matrix[0]])
        s += " [%s],\n" % ",".join([("%.4f" % n).rjust(10)
                                    for n in self.matrix[1]])
        s += " [%s],\n" % ",".join([("%.4f" % n).rjust(10)
                                    for n in self.matrix[2]])
        s += " [%s]]" % ",".join([("%.4f" % n).rjust(10)
                                  for n in self.matrix[3]])
        return s


class Rotation(Transformation):
    """The ``Rotation`` represents a 4x4 rotation matrix.

    The class contains methods for converting rotation matrices to axis-angle
    representations, Euler angles, and quaternions.
    """

    # used for Euler angles: to map rotation type and axes to tuples of inner
    # axis, parity, repetition, frame
    _SPEC2TUPLE = {
        'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
        'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
        'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
        'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
        'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
        'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
        'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
        'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

    _NEXT_SPEC = [1, 2, 0, 1]

    @classmethod
    def from_basis_vectors(cls, xaxis, yaxis):
        """Creates a ``Rotation`` from basis vectors (= orthonormal vectors).

        Computes a change of basis rotation from world XY to the coordinate
        system (frame) defined by the 2 basis vectors.

        Args:
            xaxis (:obj:`iterable` oof :obj:`float`): The x-axis of the frame.
            yaxis (:obj:`iterable` oof :obj:`float`): The y-axis of the frame.
        """
        xaxis = normalize_vector(list(xaxis))
        yaxis = normalize_vector(list(yaxis))
        zaxis = cross_vectors(xaxis, yaxis)
        yaxis = cross_vectors(zaxis, xaxis)  # correction

        R = cls()
        R.matrix[0][0], R.matrix[1][0], R.matrix[2][0] = xaxis
        R.matrix[0][1], R.matrix[1][1], R.matrix[2][1] = yaxis
        R.matrix[0][2], R.matrix[1][2], R.matrix[2][2] = zaxis
        return R

    @classmethod
    def from_quaternion(cls, quaternion):
        """Calculates a ``Rotation`` from quaternion coefficients.

        Args:
            quaternion (:obj:`iterable` of :obj:`float`): Four numbers that
                represents the four coefficient values of a quaternion.

        Example:
            q = [0.945, -0.021, -0.125, 0.303]
            R = Rotation.from_quaternion(q)
        """

        q = quaternion
        n = q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2  # dot product

        epsilon = 1.0e-15

        if n < epsilon:
            return cls()

        q = [v * math.sqrt(2.0 / n) for v in q]
        q = [[q[i] * q[j] for i in range(4)]
             for j in range(4)]  # outer_product

        rotation = cls.from_matrix([
            [1.0 - q[2][2] - q[3][3], q[1][2] - q[3][0], q[1][3] + q[2][0],
             0.0],
            [q[1][2] + q[3][0], 1.0 - q[1][1] - q[3][3], q[2][3] - q[1][0],
             0.0],
            [q[1][3] - q[2][0], q[2][3] + q[1][0], 1.0 - q[1][1] - q[2][2],
             0.0],
            [0.0, 0.0, 0.0, 1.0]])
        return rotation

    @classmethod
    def from_axis_angle_vector(cls, axis_angle_vector):
        """Calculates a ``Rotation`` from an axis-angle vector.

        Args:
            axis_angle_vector (:obj:`iterable` of :obj:`float`): Three numbers
                that represent the axis of rotation and angle of rotation
                through the vector's magnitude.

        Example:
            aav1 = [-0.043, -0.254, 0.617]
            R = Rotation.from_axis_angle_vector(aav1)
            aav2 = R.axis_angle_vector
            print(allclose(aav1, aav2))
        """

        axis_angle_vector = list(axis_angle_vector)
        angle = length_vector(axis_angle_vector)
        return cls.from_axis_and_angle(axis_angle_vector, angle)

    @classmethod
    def from_axis_and_angle(cls, axis, angle, point=None):
        """Calculates a ``Rotation`` from an rotation axis and an angle or a
            ``Transformation`` if the optional point of rotation is provided.

        Note:
            The rotation is based on the right hand rule, i.e. anti-clockwise
            if the axis of rotation points towards the observer.

        Args:
            axis (:obj:`iterable` of :obj:`float`): Three numbers that
                represent the axis of rotation
            angle (:obj:`float`): The rotaion angle in radians.
            point (:obj:`iterable` of :obj:`float`, optional): A point to
                perform a rotation around an origin other than (0, 0, 0).

        Example:
            R = Rotation.from_axis_and_angle([-0.043, -0.254, 0.617], math.pi)

        Returns:
            (:class:`Rotation`): If point is None.
            (:class:`Transformation`): If optional point is not None.
        """

        axis = list(axis)
        if length_vector(axis):
            axis = normalize_vector(axis)

        sina = math.sin(angle)
        cosa = math.cos(angle)

        R = [[cosa, 0.0, 0.0], [0.0, cosa, 0.0], [0.0, 0.0, cosa]]

        outer_product = [[axis[i] * axis[j] *
                          (1.0 - cosa) for i in range(3)] for j in range(3)]
        R = [[R[i][j] + outer_product[i][j]
              for i in range(3)] for j in range(3)]

        axis = scale_vector(axis, sina)
        m = [[0.0, -axis[2], axis[1]],
             [axis[2], 0.0, -axis[0]],
             [-axis[1], axis[0], 0.0]]

        rotation = cls()
        for i in range(3):
            for j in range(3):
                rotation.matrix[i][j] = R[i][j] + m[i][j]

        if point is not None:
            # rotation about axis, angle AND point includes also translation
            t = subtract_vectors(point, rotation.transform(point))
            rotation.matrix[0][3] = t[0]
            rotation.matrix[1][3] = t[1]
            rotation.matrix[2][3] = t[2]
            return Transformation.from_matrix(rotation.matrix)
        else:
            return rotation

    @classmethod
    def from_euler_angles(cls, euler_angles, static=True, axes='xyz'):
        """Calculates a ``Rotation`` from Euler angles.

        In 3D space any orientation can be achieved by composing three
        elemental rotations, rotations about the axes (x,y,z) of a coordinate
        system. A triple of Euler angles can be interpreted in 24 ways, which
        depends on if the rotations are applied to a static (extrinsic) or
        rotating (intrinsic) frame and the order of axes.

        Args:
            euler_angles(:obj:`iterable` of :obj:`float`): Three numbers that
                represent the axis of rotation
            static(:obj:`bool`, optional): If true the rotations are applied to
                a static frame. If not, to a rotational. Defaults to true.
            axes(:obj:`str`, optional): A 3 character string specifying order
                of the axes. Defaults to 'xyz'.

        Example:
            ea1 = 1.4, 0.5, 2.3
            R = Rotation.from_euler_angles(ea1, static = True, 'xyz')
            ea2 = R.euler_angles(static = True, axes = 'xyz')
            print(allclose(ea1, ea2))
        """

        ai, aj, ak = euler_angles

        if static:
            firstaxis, parity, repetition, frame = cls._SPEC2TUPLE["s" + axes]
        else:
            firstaxis, parity, repetition, frame = cls._SPEC2TUPLE["r" + axes]

        i = firstaxis
        j = cls._NEXT_SPEC[i + parity]
        k = cls._NEXT_SPEC[i - parity + 1]

        if frame:
            ai, ak = ak, ai
        if parity:
            ai, aj, ak = -ai, -aj, -ak

        si, sj, sk = math.sin(ai), math.sin(aj), math.sin(ak)
        ci, cj, ck = math.cos(ai), math.cos(aj), math.cos(ak)
        cc, cs = ci * ck, ci * sk
        sc, ss = si * ck, si * sk

        M = [[1. if x == y else 0. for x in range(4)] for y in range(4)]
        if repetition:
            M[i][i] = cj
            M[i][j] = sj * si
            M[i][k] = sj * ci
            M[j][i] = sj * sk
            M[j][j] = -cj * ss + cc
            M[j][k] = -cj * cs - sc
            M[k][i] = -sj * ck
            M[k][j] = cj * sc + cs
            M[k][k] = cj * cc - ss
        else:
            M[i][i] = cj * ck
            M[i][j] = sj * sc - cs
            M[i][k] = sj * cc + ss
            M[j][i] = cj * sk
            M[j][j] = sj * ss + cc
            M[j][k] = sj * cs - sc
            M[k][i] = -sj
            M[k][j] = cj * si
            M[k][k] = cj * ci
        return Rotation.from_matrix(M)

    @property
    def quaternion(self):
        """Returns the 4 quaternion coefficients from the rotation.
        """
        m = self.matrix

        qw, qx, qy, qz = 0, 0, 0, 0
        trace = m[0][0] + m[1][1] + m[2][2]

        if trace > 0.0:
            s = (0.5 / math.sqrt(trace + 1.0))
            qw = 0.25 / s
            qx = (m[2][1] - m[1][2]) * s
            qy = (m[0][2] - m[2][0]) * s
            qz = (m[1][0] - m[0][1]) * s

        elif ((m[0][0] > m[1][1]) and (m[0][0] > m[2][2])):
            s = 2.0 * math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2])
            qw = (m[2][1] - m[1][2]) / s
            qx = 0.25 * s
            qy = (m[0][1] + m[1][0]) / s
            qz = (m[0][2] + m[2][0]) / s

        elif (m[1][1] > m[2][2]):
            s = 2.0 * math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2])
            qw = (m[0][2] - m[2][0]) / s
            qx = (m[0][1] + m[1][0]) / s
            qy = 0.25 * s
            qz = (m[1][2] + m[2][1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1])
            qw = (m[1][0] - m[0][1]) / s
            qx = (m[0][2] + m[2][0]) / s
            qy = (m[1][2] + m[2][1]) / s
            qz = 0.25 * s

        return [qw, qx, qy, qz]

    @property
    def axis_and_angle(self):
        """Returns the axis and the angle of rotation.
        """
        epsilon = 0.01  # margin to allow for rounding errors
        epsilon2 = 0.1  # margin to distinguish between 0 and 180 degrees

        M = self.matrix

        if ((math.fabs(M[0][1] - M[1][0]) < epsilon) and
            (math.fabs(M[0][2] - M[2][0]) < epsilon) and
                (math.fabs(M[1][2] - M[2][1]) < epsilon)):

            # Singularity found.
            # First check for identity matrix which must have + 1 for all terms
            # in leading diagonal and zero in other terms
            if ((math.fabs(M[0][1] + M[1][0]) < epsilon2) and
                (math.fabs(M[0][2] + M[2][0]) < epsilon2) and
                (math.fabs(M[1][2] + M[2][1]) < epsilon2) and
                    (math.fabs(M[0][0] + M[1][1] + M[2][2] - 3) < epsilon2)):
                    # this singularity is identity matrix so angle = 0
                return [0, 0, 0], 0
            else:
                # otherwise this singularity is angle = 180
                angle = math.pi
                xx = (M[0][0] + 1) / 2
                yy = (M[1][1] + 1) / 2
                zz = (M[2][2] + 1) / 2
                xy = (M[0][1] + M[1][0]) / 4
                xz = (M[0][2] + M[2][0]) / 4
                yz = (M[1][2] + M[2][1]) / 4
                root_half = math.sqrt(0.5)
                if ((xx > yy) and (xx > zz)):
                    if (xx < epsilon):
                        axis = [0, root_half, root_half]
                    else:
                        x = math.sqrt(xx)
                        axis = [x, xy / x, xz / x]
                elif (yy > zz):
                    if (yy < epsilon):
                        axis = [root_half, 0, root_half]
                    else:
                        y = math.sqrt(yy)
                        axis = [xy / y, y, yz / y]
                else:
                    if (zz < epsilon):
                        axis = [root_half, root_half, 0]
                    else:
                        z = math.sqrt(zz)
                        axis = [xz / z, yz / z, z]

                return axis, angle

        s = math.sqrt(
            (M[2][1] - M[1][2]) * (M[2][1] - M[1][2]) +
            (M[0][2] - M[2][0]) * (M[0][2] - M[2][0]) +
            (M[1][0] - M[0][1]) * (M[1][0] - M[0][1]))

        # prevent divide by zero][should not happen if matrix is orthogonal and
        # should be
        # caught by singularity test above][but I've left it in just in case
        if (math.fabs(s) < 0.001):
            s = 1
        angle = math.acos((M[0][0] + M[1][1] + M[2][2] - 1) / 2)

        x = (M[2][1] - M[1][2]) / s
        y = (M[0][2] - M[2][0]) / s
        z = (M[1][0] - M[0][1]) / s

        return [x, y, z], angle

    @property
    def axis_angle_vector(self):
        """Returns the axis-angle vector of the rotation.

        Example:
            aav1 = [-0.043, -0.254, 0.617]
            R = Rotation.from_axis_angle_vector(aav1)
            aav2 = R.axis_angle_vector
            print(allclose(aav1, aav2))
        """
        axis, angle = self.axis_and_angle
        return [angle * axis[0], angle * axis[1], angle * axis[2]]

    def euler_angles(self, static=True, axes='xyz'):
        """Returns Euler angles from the `Rotation` according to specified axis
            sequence and rotation type.

        Args:
            static(:obj:`bool`, optional): If true the rotations are applied to
                a static frame. If not, to a rotational. Defaults to true.
            axes(:obj:`str`, optional): A 3 character string specifying order
                of the axes. Defaults to 'xyz'.

        Returns:
            (:obj:`list` of :obj:`float`): The 3 Euler angles.

        Example:
            ea1 = 1.4, 0.5, 2.3
            R = Rotation.from_euler_angles(ea1, static = True, 'xyz')
            ea2 = R.euler_angles(static = True, axes = 'xyz')
            print(allclose(ea1, ea2))
        """

        if static:
            firstaxis, parity, repetition, frame = self._SPEC2TUPLE["s" + axes]
        else:
            firstaxis, parity, repetition, frame = self._SPEC2TUPLE["r" + axes]

        i = firstaxis
        j = self._NEXT_SPEC[i + parity]
        k = self._NEXT_SPEC[i - parity + 1]

        M = self.matrix

        if repetition:
            sy = math.sqrt(M[i][j] * M[i][j] + M[i][k] * M[i][k])
            if sy > _EPS:
                ax = math.atan2(M[i][j], M[i][k])
                ay = math.atan2(sy, M[i][i])
                az = math.atan2(M[j][i], -M[k][i])
            else:
                ax = math.atan2(-M[j][k], M[j][j])
                ay = math.atan2(sy, M[i][i])
                az = 0.0
        else:
            cy = math.sqrt(M[i][i] * M[i][i] + M[j][i] * M[j][i])
            if cy > _EPS:
                ax = math.atan2(M[k][j], M[k][k])
                ay = math.atan2(-M[k][i], cy)
                az = math.atan2(M[j][i], M[i][i])
            else:
                ax = math.atan2(-M[j][k], M[j][j])
                ay = math.atan2(-M[k][i], cy)
                az = 0.0

        if parity:
            ax, ay, az = -ax, -ay, -az
        if frame:
            ax, az = az, ax

        return [ax, ay, az]

    @property
    def basis_vectors(self):

        xaxis = [self.matrix[0][0], self.matrix[1][0], self.matrix[2][0]]
        yaxis = [self.matrix[0][1], self.matrix[1][1], self.matrix[2][1]]
        zaxis = [self.matrix[0][2], self.matrix[1][2], self.matrix[2][2]]

        print("--")
        print("1", cross_vectors(xaxis, yaxis))
        print("2", zaxis)
        print("--")
        return [xaxis, yaxis]


class Translation(Transformation):
    """Creates a translation transformation.

    Args:
        translation (:obj:`iterable` of :obj:`float`): a list of 3 numbers
            defining the translation in x, y, and z.
    """

    def __init__(self, translation):
        super(Translation, self).__init__()
        self.matrix[0][3] = float(translation[0])
        self.matrix[1][3] = float(translation[1])
        self.matrix[2][3] = float(translation[2])


class Scale(Transformation):
    """Creates a scaling transformation.

    Args:
        xfactor (float): Scaling factor along x-axis.
        yfactor (float): Scaling factor along y-axis.
        zfactor (float): Scaling factor along z-axis.
    """

    def __init__(self, xfactor, yfactor, zfactor):
        super(Scale, self).__init__()
        self.matrix[0][0] = float(xfactor)
        self.matrix[1][1] = float(yfactor)
        self.matrix[2][2] = float(zfactor)


class Reflection(Transformation):
    """Creates a ``Reflection`` that mirrors points at a plane, defined by
        point and normal vector.

        Args:
        point (:obj:`iterable` of :obj:`float`): A list of 3 numbers.
        normal (:obj:`iterable` of :obj:`float`): A list of 3 numbers.

    """

    def __init__(self, point, normal):

        reflection = self()

        normal = normalize_vector((list(normal)))

        for i in range(3):
            for j in range(3):
                reflection.matrix[i][j] -= 2.0 * normal[i] * normal[j]

        for i in range(3):
            reflection.matrix[i][3] = 2 * dot_vectors(point, normal) *\
                normal[i]
        return reflection

    @classmethod
    def from_frame(cls, frame):
        """Creates a ``Reflection`` that mirrors points at the given frame.

        Args:
            frame(:class:`Frame`)
        """
        return cls(frame.point, frame.normal)


class Projection(Transformation):
    """Creates a ``Projection`` that projects onto a plane defined by point and
    normal.

    Using either perspective point, projection direction, or none of both.

    Example:
        from random import random as rnd
        point = [rnd() - 0.5 for i in range(3)]
        normal = [rnd() - 0.5 for i in range(3)]
        direct = [rnd() - 0.5 for i in range(3)]
        persp = [rnd() - 0.5 for i in range(3)]

        print("point =", point)
        print("normal =", normal)
        print("direct =", direct)
        print("persp =", persp)

        P0 = Projection(point, normal)
        P1 = Projection(point, normal, direction=direct)
        P2 = Projection(point, normal, perspective=persp)
        print(P0)
        print(P1)
        print(P2)

    """

    def __init__(self, point, normal, direction=None, perspective=None):

        M = [[1. if i == j else 0. for i in range(4)] for j in range(4)]

        normal = normalize_vector(normal)

        if perspective is not None:
            # perspective projection

            M[0][0] = M[1][1] = M[2][2] = dot_vectors(
                subtract_vectors(perspective, point), normal)

            for j in range(3):
                for i in range(3):
                    M[i][j] -= perspective[i] * normal[j]

            M[0][3], M[1][3], M[2][3] = scale_vector(
                perspective, dot_vectors(point, normal))
            for i in range(3):
                M[3][i] -= normal[i]
            M[3][3] = dot_vectors(perspective, normal)

        elif direction is not None:
            # parallel projection
            scale = dot_vectors(direction, normal)
            for j in range(3):
                for i in range(3):
                    M[i][j] -= direction[i] * normal[j] / scale

            M[0][3], M[1][3], M[2][3] = scale_vector(
                direction, dot_vectors(point, normal) / scale)
        else:
            # orthogonal projection
            for j in range(3):
                for i in range(3):
                    M[i][j] -= normal[i] * normal[j]  # outer_product

            M[0][3], M[1][3], M[2][3] = scale_vector(
                normal, dot_vectors(point, normal))

        self.matrix = M


class Shear(Transformation):
    """Shear(plane: Plane, x: Vector3d, y: Vector3d, z: Vector3d) -> Transform

    Constructs a Shear transformation.
    plane: Base plane for shear.
    x: Shearing vector along plane x-axis.
    y: Shearing vector along plane y-axis.
    z: Shearing vector along plane z-axis.
    Returns: A transformation matrix which shear geometry.
    """

    def __init__(self, angle, direction, point, normal):
        """Creates a transformation to shear by angle along direction vector on
            the shear plane.

        A point P is transformed by the shear matrix into P" such that
        the vector P-P" is parallel to the direction vector and its extent is
        given by the angle of P-P'-P", where P' is the orthogonal projection
        of P onto the shear plane (defined by point and normal).

        Args:
            angle (:obj:`float`): The angle in radians.
            direction (:obj:`iterable` of :obj:`float`): The direction vector
                as list of 3 numbers. It must be orthogonal to the normal
                vector.
            point (:obj:`iterable` of :obj:`float`): The point of the shear
                plane as list of 3 numbers.
            normal (:obj:`iterable` of :obj:`float`): The normal of the shear
                plane as list of 3 numbers.

        Example:
            angle = (random.random() - 0.5) * 4*math.pi
            direct = numpy.random.random(3) - 0.5
            point = numpy.random.random(3) - 0.5
            normal = numpy.cross(direct, numpy.random.random(3))
            S = shear_matrix(angle, direct, point, normal)
            numpy.allclose(1, numpy.linalg.det(S))
        """

        normal = normalize_vector(normal)
        direction = normalize_vector(direction)

        if math.fabs(dot_vectors(normal, direction)) > _EPS:
            raise ValueError('Direction and normal vectors are not orthogonal')

        angle = math.tan(angle)
        M = [[1. if i == j else 0. for i in range(4)] for j in range(4)]

        for j in range(3):
            for i in range(3):
                M[i][j] += angle * direction[i] * normal[j]

        M[0][3], M[1][3], M[2][3] = scale_vector(
            direction, -angle * dot_vectors(point, normal))

        self.matrix = M


if __name__ == "__main__":

    from compas_fab.fab.geometry import Frame

    aav1 = [-0.043, -0.254, 0.617]
    R = Rotation.from_axis_angle_vector(aav1)
    aav2 = R.axis_angle_vector
    print("aav1 == aav2", allclose(aav1, aav2))
    print("det", determinant(R.matrix))
    print("=================")

    args = True, 'xyz'
    ea1 = [2.03, -0.1, 0.5]
    R = Rotation.from_euler_angles(ea1, *args)
    print(R)
    print("det", determinant(R.matrix))
    ea2 = R.euler_angles(*args)
    print(allclose(ea1, ea2))

    print("=================")
    print(">>", R.axis_angle_vector)
    print(R.basis_vectors)
    print("=================")

    from random import random as rnd
    point = [rnd() - 0.5 for i in range(3)]
    normal = [rnd() - 0.5 for i in range(3)]
    direct = [rnd() - 0.5 for i in range(3)]
    persp = [rnd() - 0.5 for i in range(3)]

    point = [-0.4115985042852909, -0.1440252439994525, -0.35887224226687087]
    normal = [0.2648560337872353, 0.17956931561274203, 0.44275624449933515]
    direct = [0.2723206499809814, -0.40163729179666474, 0.21388354572922286]
    persp = [-0.49300110873599023, -0.39194105941568413, 0.49221271116515697]

    P0 = Projection(point, normal)
    P1 = Projection(point, normal, direction=direct)
    P2 = Projection(point, normal, perspective=persp)
    print(P0)
    print(P1)
    print(P2)
    angles, translation, scale, shear, perspective = P2.decompose()
    print(determinant(P2.matrix))

    print("=================")
    angle = 0.234
    point = [-0.4115985042852909, -0.1440252439994525, -0.35887224226687087]
    normal = [0.2648560337872353, 0.17956931561274203, 0.44275624449933515]
    direction = cross_vectors(
        normal, [
            0.2723206499809814, -0.40163729179666474, 0.21388354572922286])
    Sh = Shear(angle, direction, point, normal)
    print(Sh)
    print(determinant(Sh.matrix))

    scale, shear, angles, translate, perspective = Sh.decompose()

    print("scale", scale)
    print("share", shear)
    print("angles", angles)
    print("translate", translate)
    print("perspective", perspective)

    print("=================")

    f1 = Frame.worldXY()
    f2 = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])

    R = Rotation.from_frame(f2)
    Sc = Scale(1, 2, 3)
    T = Translation([4, 5, 6])
    Sh = Shear(angle, direction, point, normal)
    P = Projection(point, normal, perspective=persp)
    M = P * Sh * T * Sc * R
    print("M", M)

    angles, translation, scale, shear, perspective = M.decompose()

    print("scale", scale)
    print("share", shear)
    print("angles", angles)
    print("translate", translate)
    print("perspective", perspective)
