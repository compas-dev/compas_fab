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

from compas_fab.fab.utilities.numbers import allclose

__author__ = ['Romana Rust <rust@arch.ethz.ch>', ]
__license__ = 'MIT License'
__email__ = 'rust@arch.ethz.ch'

__all__ = [
    'determinant',
    'inverse',
    'identity_matrix',
    'matrix_from_frame',
    'matrix_from_euler_angles',
    'euler_angles_from_matrix',
    'matrix_from_axis_and_angle',
    'matrix_from_axis_angle_vector',
    'axis_and_angle_from_matrix',
    'axis_angle_vector_from_matrix',
    'matrix_from_quaternion',
    'quaternion_from_matrix',
    'matrix_from_basis_vectors',
    'basis_vectors_from_matrix',
    'matrix_from_translation',
    'translation_from_matrix',
    'matrix_from_perspective_entries',
    'matrix_from_shear_entries',
    'matrix_from_shear',
    'matrix_from_scale_factors',
    'compose_matrix',
    'decompose_matrix',
    'Transformation',
    'Rotation',
    'Translation',
    'Scale',
    'Reflection',
    'Projection',
    'Shear'
]

# TODO: need to be moved ideally to compas.geometry.basic


# epsilon for testing whether a number is close to zero
_EPS = 1e-16

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


def determinant(M, check=True):
    """Calculates the determinant of a square matrix M.

    Args:
        M (:obj:`list` of :obj:`list` of :obj:`float`): The square matrix of \
            any dimension.
        check (bool): If true checks if matrix is squared. Defaults to True.

    Raises:
        ValueError: If matrix is not a square matrix.

    Returns:
        (:obj:`float`): The determinant.
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
        M (:obj:`list` of :obj:`list` of :obj:`float`): The square
            matrix of any dimension.

    Raises:
        ValueError: If the matrix is not squared
        ValueError: If the matrix is singular.
        ValueError: If the matrix is not invertible.

    Returns:
        (:obj:`list` of :obj:`list` of :obj:`float`): The inverted matrix.

    Example:
        >>> f = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
        >>> T = matrix_from_frame(f)
        >>> I = multiply_matrices(T * inverse(T))
    """

    detM = determinant(M)  # raises ValueError if matrix is not squared

    if detM == 0:
        ValueError("The matrix is singular.")

    dim = len(M)

    # create identity I and copy M into C
    I = identity_matrix(dim)
    C = [[float(M[j][i]) for i in range(dim)] for j in range(dim)]

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


def identity_matrix(dim):
    return [[1. if i == j else 0. for i in range(dim)] for j in range(dim)]


def matrix_from_frame(frame):
    """Computes a change of basis transformation from world XY to the frame.

    Args:
        frame (:class:`Frame`): a frame describing the targeted Cartesian
            coordinate system

    Example:
        >>> f = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
        >>> T = matrix_from_frame(f)
    """

    M = identity_matrix(4)
    M[0][0], M[1][0], M[2][0] = frame.xaxis
    M[0][1], M[1][1], M[2][1] = frame.yaxis
    M[0][2], M[1][2], M[2][2] = frame.zaxis
    M[0][3], M[1][3], M[2][3] = frame.point
    return M


def matrix_from_euler_angles(euler_angles, static=True, axes='xyz'):
    """Calculates a rotation matrix from Euler angles.

    In 3D space any orientation can be achieved by composing three elemental
    rotations, rotations about the axes (x,y,z) of a coordinate system. A
    triple of Euler angles can be interpreted in 24 ways, which depends on if
    the rotations are applied to a static (extrinsic) or rotating (intrinsic)
    frame and the order of axes.

    Args:
        euler_angles(:obj:`list` of :obj:`float`): Three numbers that represent
            the angles of rotations about the defined axes.
        static(:obj:`bool`, optional): If true the rotations are applied to a
            static frame. If not, to a rotational. Defaults to true.
        axes(:obj:`str`, optional): A 3 character string specifying order of
            the axes. Defaults to 'xyz'.

    Example:
        >>> ea1 = 1.4, 0.5, 2.3
        >>> args = True, 'xyz'
        >>> R = matrix_from_euler_angles(ea1, *args)
        >>> ea2 = euler_angles_from_matrix(*args)
        >>> allclose(ea1, ea2)
        True
    """

    global _SPEC2TUPLE
    global _NEXT_SPEC

    ai, aj, ak = euler_angles

    if static:
        firstaxis, parity, repetition, frame = _SPEC2TUPLE["s" + axes]
    else:
        firstaxis, parity, repetition, frame = _SPEC2TUPLE["r" + axes]

    i = firstaxis
    j = _NEXT_SPEC[i + parity]
    k = _NEXT_SPEC[i - parity + 1]

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

    return M


def euler_angles_from_matrix(M, static=True, axes='xyz'):
    """Returns Euler angles from the rotation matrix M according to specified
        axis sequence and type of rotation.

    Args:
        M (:obj:`list` of :obj:`list` of :obj:`float`): The 3x3 or 4x4 matrix
            in row-major order.
        static(:obj:`bool`, optional): If true the rotations are applied to a
            static frame. If not, to a rotational. Defaults to True.
        axes(:obj:`str`, optional): A 3 character string specifying order of
            the axes. Defaults to 'xyz'.

    Returns:
        (:obj:`list` of :obj:`float`): The 3 Euler angles.

    Example:
        >>> ea1 = 1.4, 0.5, 2.3
        >>> args = True, 'xyz'
        >>> R = matrix_from_euler_angles(ea1, *args)
        >>> ea2 = euler_angles_from_matrix(*args)
        >>> allclose(ea1, ea2)
        True
    """

    global _SPEC2TUPLE
    global _NEXT_SPEC
    global _EPS

    if static:
        firstaxis, parity, repetition, frame = _SPEC2TUPLE["s" + axes]
    else:
        firstaxis, parity, repetition, frame = _SPEC2TUPLE["r" + axes]

    i = firstaxis
    j = _NEXT_SPEC[i + parity]
    k = _NEXT_SPEC[i - parity + 1]

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


def matrix_from_axis_and_angle(axis, angle, point=[0, 0, 0]):
    """Calculates a rotation matrix from an rotation axis, an angle and an
        optional point of rotation.

    Note:
        The rotation is based on the right hand rule, i.e. anti-clockwise
        if the axis of rotation points towards the observer.

    Args:
        axis (:obj:`list` of :obj:`float`): Three numbers that
            represent the axis of rotation
        angle (:obj:`float`): The rotaion angle in radians.
        point (:obj:`list` of :obj:`float`, optional): A point to
            perform a rotation around an origin other than [0, 0, 0].

    Returns:
        (:obj:`list` of :obj:`list` of :obj:`float`): The matrix.

    Example:
        >>> axis1 = normalize_vector([-0.043, -0.254, 0.617])
        >>> angle1 = 0.1
        >>> R = matrix_from_axis_and_angle(axis1, angle1)
        >>> axis2, angle2 = axis_and_angle_from_matrix(R)
        >>> allclose(axis1, axis2)
        True
        >>> allclose([angle1], [angle2])
        True
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

    M = [[1. if x == y else 0. for x in range(4)] for y in range(4)]
    for i in range(3):
        for j in range(3):
            R[i][j] += m[i][j]
            M[i][j] = R[i][j]

    # rotation about axis, angle AND point includes also translation
    t = subtract_vectors(point, multiply_matrix_vector(R, point))
    M[0][3] = t[0]
    M[1][3] = t[1]
    M[2][3] = t[2]

    return M


def matrix_from_axis_angle_vector(axis_angle_vector, point=[0, 0, 0]):
    """Calculates a rotation matrix from an axis-angle vector.

    Args:
        axis_angle_vector (:obj:`list` of :obj:`float`): Three numbers
            that represent the axis of rotation and angle of rotation
            through the vector's magnitude.
        point (:obj:`list` of :obj:`float`, optional): A point to
            perform a rotation around an origin other than [0, 0, 0].

    Example:
        >>> aav1 = [-0.043, -0.254, 0.617]
        >>> R = matrix_from_axis_angle_vector(aav1)
        >>> aav2 = axis_and_angle_from_matrix(R)
        >>> allclose(aav1, aav2)
        True
    """
    axis = list(axis_angle_vector)
    angle = length_vector(axis_angle_vector)
    return matrix_from_axis_and_angle(axis, angle, point)


def axis_and_angle_from_matrix(M):
    """Returns the axis and the angle of the rotation matrix M.


    """
    epsilon = 0.01  # margin to allow for rounding errors
    epsilon2 = 0.1  # margin to distinguish between 0 and 180 degrees

    if ((math.fabs(M[0][1] - M[1][0]) < epsilon) and
        (math.fabs(M[0][2] - M[2][0]) < epsilon) and
            (math.fabs(M[1][2] - M[2][1]) < epsilon)):

        if ((math.fabs(M[0][1] + M[1][0]) < epsilon2) and
            (math.fabs(M[0][2] + M[2][0]) < epsilon2) and
            (math.fabs(M[1][2] + M[2][1]) < epsilon2) and
                (math.fabs(M[0][0] + M[1][1] + M[2][2] - 3) < epsilon2)):
            return [0, 0, 0], 0
        else:
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

    if (math.fabs(s) < 0.001):
        s = 1
    angle = math.acos((M[0][0] + M[1][1] + M[2][2] - 1) / 2)

    x = (M[2][1] - M[1][2]) / s
    y = (M[0][2] - M[2][0]) / s
    z = (M[1][0] - M[0][1]) / s

    return [x, y, z], angle


def axis_angle_vector_from_matrix(M):
    """Returns the axis-angle vector of the rotation matrix M.
    """
    axis, angle = axis_and_angle_from_matrix(M)
    return scale_vector(axis, angle)


def matrix_from_quaternion(quaternion):
    """Calculates a ``Rotation`` from quaternion coefficients.

    Args:
        quaternion (:obj:`list` of :obj:`float`): Four numbers that
            represents the four coefficient values of a quaternion.

    Raises:
        ValueError: If quaternion is invalid.

    Example:
        >>> q1 = [0.945, -0.021, -0.125, 0.303]
        >>> R = matrix_from_quaternion(q1)
        >>> q2 = quaternion_from_matrix(R)
        >>> allclose(q1, q2)
        True
    """
    q = quaternion
    n = q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2  # dot product

    epsilon = 1.0e-15

    if n < epsilon:
        raise ValueError("Invalid quaternion, dot product must be != 0.")

    q = [v * math.sqrt(2.0 / n) for v in q]
    q = [[q[i] * q[j] for i in range(4)]
         for j in range(4)]  # outer_product

    rotation = [
        [1.0 - q[2][2] - q[3][3], q[1][2] - q[3][0], q[1][3] + q[2][0], 0.0],
        [q[1][2] + q[3][0], 1.0 - q[1][1] - q[3][3], q[2][3] - q[1][0], 0.0],
        [q[1][3] - q[2][0], q[2][3] + q[1][0], 1.0 - q[1][1] - q[2][2], 0.0],
        [0.0, 0.0, 0.0, 1.0]]
    return rotation


def quaternion_from_matrix(M):
    """Returns the 4 quaternion coefficients from a rotation matrix.

    Args:
        M (:obj:`list` of :obj:`list` of :obj:`float`): The rotation matrix.

    Returns:
        (:obj:`list` of :obj:`float`): The quaternion coefficients.

    Example:
        >>> q1 = [0.945, -0.021, -0.125, 0.303]
        >>> R = matrix_from_quaternion(q)
        >>> q2 = quaternion_from_matrix(R)
        >>> allclose(q1, q2)
        True
    """

    qw, qx, qy, qz = 0, 0, 0, 0
    trace = M[0][0] + M[1][1] + M[2][2]

    if trace > 0.0:
        s = (0.5 / math.sqrt(trace + 1.0))
        qw = 0.25 / s
        qx = (M[2][1] - M[1][2]) * s
        qy = (M[0][2] - M[2][0]) * s
        qz = (M[1][0] - M[0][1]) * s

    elif ((M[0][0] > M[1][1]) and (M[0][0] > M[2][2])):
        s = 2.0 * math.sqrt(1.0 + M[0][0] - M[1][1] - M[2][2])
        qw = (M[2][1] - M[1][2]) / s
        qx = 0.25 * s
        qy = (M[0][1] + M[1][0]) / s
        qz = (M[0][2] + M[2][0]) / s

    elif (M[1][1] > M[2][2]):
        s = 2.0 * math.sqrt(1.0 + M[1][1] - M[0][0] - M[2][2])
        qw = (M[0][2] - M[2][0]) / s
        qx = (M[0][1] + M[1][0]) / s
        qy = 0.25 * s
        qz = (M[1][2] + M[2][1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + M[2][2] - M[0][0] - M[1][1])
        qw = (M[1][0] - M[0][1]) / s
        qx = (M[0][2] + M[2][0]) / s
        qy = (M[1][2] + M[2][1]) / s
        qz = 0.25 * s

    return [qw, qx, qy, qz]


def matrix_from_basis_vectors(xaxis, yaxis):
    """Creates a rotation matrix from basis vectors (= orthonormal vectors).

    Args:
        xaxis (:obj:`list` oof :obj:`float`): The x-axis of the frame.
        yaxis (:obj:`list` oof :obj:`float`): The y-axis of the frame.

    Example:
        >>> xaxis = [0.68, 0.68, 0.27]
        >>> yaxis = [-0.67, 0.73, -0.15]
        >>> R = matrix_from_basis_vectors(xaxis, yaxis)
    """
    xaxis = normalize_vector(list(xaxis))
    yaxis = normalize_vector(list(yaxis))
    zaxis = cross_vectors(xaxis, yaxis)
    yaxis = cross_vectors(zaxis, xaxis)  # correction

    R = identity_matrix(4)
    R[0][0], R[1][0], R[2][0] = xaxis
    R[0][1], R[1][1], R[2][1] = yaxis
    R[0][2], R[1][2], R[2][2] = zaxis
    return R


def basis_vectors_from_matrix(R):
    """Returns the basis vectors from the rotation matrix R.

    Raises:
        ValueError: If rotation matrix is invalid.

    Example:
        >>> f = Frame([0, 0, 0], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
        >>> R = matrix_from_frame(f)
        >>> xaxis, yaxis = basis_vectors_from_matrix(R)
    """
    xaxis = [R[0][0], R[1][0], R[2][0]]
    yaxis = [R[0][1], R[1][1], R[2][1]]
    zaxis = [R[0][2], R[1][2], R[2][2]]
    if not allclose(zaxis, cross_vectors(xaxis, yaxis)):
        raise ValueError("Matrix is invalid rotation matrix.")
    else:
        return [xaxis, yaxis]


def matrix_from_translation(translation):
    """Returns a translation matrix.

    Args:
        translation (:obj:`list` of :obj:`float`): a list of 3 numbers defining
            the translation in x, y, and z.

    Example:
        >>> T = matrix_from_translation([1, 2, 3])
    """
    M = identity_matrix(4)
    M[0][3] = float(translation[0])
    M[1][3] = float(translation[1])
    M[2][3] = float(translation[2])
    return M


def translation_from_matrix(M):
    """Returns the 3 values of translation from the matrix M.
    """
    return [M[0][3], M[1][3], M[2][3]]


def matrix_from_perspective_entries(perspective):
    """Returns a matrix from perspective entries.

    Args:
        values(:obj:`list` of :obj:`float`): The 4 perspective entries of a
            matrix.
    """
    M = identity_matrix(4)
    M[3][0] = float(perspective[0])
    M[3][1] = float(perspective[1])
    M[3][2] = float(perspective[2])
    M[3][3] = float(perspective[3])
    return M


def matrix_from_shear_entries(shear_entries):
    """Returns a shear matrix from the 3 factors for x-y, x-z, and y-z axes.

    Args:
        shear_entries (:obj:`list` of :obj:`float`): The 3 shear factors for
            x-y, x-z, and y-z axes.

    Example:
        >>> Sh = matrix_from_shear_entries([1, 2, 3])
    """
    M = identity_matrix(4)
    M[0][1] = float(shear_entries[0])
    M[0][2] = float(shear_entries[1])
    M[1][2] = float(shear_entries[2])
    return M


def matrix_from_shear(angle, direction, point, normal):
    """Constructs a shear matrix by an angle along the direction vector on \
        the shear plane (defined by point and normal).

    A point P is transformed by the shear matrix into P" such that
    the vector P-P" is parallel to the direction vector and its extent is
    given by the angle of P-P'-P", where P' is the orthogonal projection
    of P onto the shear plane (defined by point and normal).

    Args:
        angle (:obj:`float`): The angle in radians.
        direction (:obj:`list` of :obj:`float`): The direction vector as
            list of 3 numbers. It must be orthogonal to the normal vector.
        point (:obj:`list` of :obj:`float`): The point of the shear plane
            as list of 3 numbers.
        normal (:obj:`list` of :obj:`float`): The normal of the shear plane
            as list of 3 numbers.

    Raises:
        ValueError: If direction and normal are not orthogonal.

    Example:
        >>> angle = 0.1
        >>> direction = [0.1, 0.2, 0.3]
        >>> point = [4, 3, 1]
        >>> normal = cross_vectors(direction, [1, 0.3, -0.1])
        >>> S = matrix_from_shear(angle, direction, point, normal)
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

    return M


def matrix_from_scale_factors(scale_factors):
    """Returns a scaling transformation.

    Args:
        scale_factors (:obj:`list` of :obj:`float`):  Three numbers defining
            the scaling factors in x, y, and z respectively.

    Example:
        >>> Sc = matrix_from_scale_factors([1, 2, 3])
    """
    M = identity_matrix(4)
    M[0][0] = float(scale_factors[0])
    M[1][1] = float(scale_factors[1])
    M[2][2] = float(scale_factors[2])
    return M


def decompose_matrix(M):
    """Calculates the components of rotation, translation, scale, shear, \
        and perspective of a given transformation matrix M.

    Returns:

        scale (:obj:`list` of :obj:`float`): The 3 scale factors in x-, y-, \
            and z-direction.

        shear (:obj:`list` of :obj:`float`): The 3 shear factors for x-y, \
            x-z, and y-z axes.

        angles (:obj:`list` of :obj:`float`): The rotation specified through \
            the 3 Euler angles about static x, y, z axes.

        translation (:obj:`list` of :obj:`float`): The 3 values of \
            translation

        perspective (:obj:`list` of :obj:`float`): The 4 perspective entries \
            of the matrix.

    Raises:
        ValueError: If matrix is singular or degenerative.

    Example:
        >>> trans1 = [1, 2, 3]
        >>> angle1 = [-2.142, 1.141, -0.142]
        >>> scale1 = [0.123, 2, 0.5]
        >>> T = matrix_from_translation(trans1)
        >>> R = matrix_from_euler_angles(angle1)
        >>> S = matrix_from_scale_factors(scale1)
        >>> M = multiply_matrices(multiply_matrices(T, R), S)
        >>> # M = compose_matrix(scale1, None, angle1, trans1, None)
        >>> scale2, shear2, angle2, trans2, persp2 = decompose_matrix(M)
        >>> allclose(scale1, scale2)
        True
        >>> allclose(angle1, angle2)
        True
        >>> allclose(trans1, trans2)
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

    return scale, shear, angles, translation, perspective


def compose_matrix(scale=None, shear=None, angles=None,
                   translation=None, perspective=None):
    """Calculates a matrix from the components of scale, shear, euler_angles, \
        translation and perspective.

    Args:
        scale (:obj:`list` of :obj:`float`): The 3 scale factors in x-, y-,
            and z-direction. Defaults to None.
        shear (:obj:`list` of :obj:`float`): The 3 shear factors for x-y,
            x-z, and y-z axes. Defaults to None.
        angles (:obj:`list` of :obj:`float`): The rotation specified
            through the 3 Euler angles about static x, y, z axes. Defaults to
            None.
        translation (:obj:`list` of :obj:`float`): The 3 values of
            translation. Defaults to None.
        perspective (:obj:`list` of :obj:`float`): The 4 perspective entries
            of the matrix. Defaults to None.

    Example:
        >>> trans1 = [1, 2, 3]
        >>> angle1 = [-2.142, 1.141, -0.142]
        >>> scale1 = [0.123, 2, 0.5]
        >>> M = compose_matrix(scale1, None, angle1, trans1, None)
        >>> scale2, shear2, angle2, trans2, persp2 = decompose_matrix(M)
        >>> allclose(scale1, scale2)
        True
        >>> allclose(angle1, angle2)
        True
        >>> allclose(trans1, trans2)
        True
    """

    M = [[1. if i == j else 0. for i in range(4)] for j in range(4)]
    if perspective is not None:
        P = matrix_from_perspective_entries(perspective)
        M = multiply_matrices(M, P)
    if translation is not None:
        T = matrix_from_translation(translation)
        M = multiply_matrices(M, T)
    if angles is not None:
        R = matrix_from_euler_angles(angles, static=True, axes="xyz")
        M = multiply_matrices(M, R)
    if shear is not None:
        Sh = matrix_from_shear_entries(shear)
        M = multiply_matrices(M, Sh)
    if scale is not None:
        Sc = matrix_from_scale_factors(scale)
        M = multiply_matrices(M, Sc)
    for i in range(4):
        for j in range(4):
            M[i][j] /= M[3][3]
    return M


class Transformation(object):
    """The ``Transformation`` represents a 4x4 transformation matrix.

    It is the base class for transformations like ``Rotation``,
    ``Translation``, ``Scale``, ``Reflection``, ``Projection`` and ``Shear``.
    The class allows to concatenate Transformations by multiplication, to
    calculate the inverse transformation and to decompose a transformation into
    its components of rotation, translation, scale, shear, and perspective.
    The matrix follows the row-major order, such that translation components
    x, y, z are in the right column of the matrix, i.e. M[0][3], M[1][3],
    M[2][3] = x, y, z

    Attributes:
        matrix (:obj:`list` of :obj:`list` of :obj:`float`)

    Examples:
        >>> T = Transformation()
        >>> f1 = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
        >>> T = Transformation.from_frame(f1)
        >>> Sc, Sh, R, T, P = T.decompose()
        >>> Tinv = T.inverse()
    """

    def __init__(self, matrix=identity_matrix(4)):
        self.matrix = matrix

    @classmethod
    def from_matrix(cls, matrix):
        """Creates a ``Transformation`` from a 4x4 two-dimensional list of \
            numbers.

        Args:
            matrix (:obj:`list` of :obj:`list` of `float`)
        """
        T = cls()
        for i in range(4):
            for j in range(4):
                T.matrix[i][j] = float(matrix[i][j])
        return T

    @classmethod
    def from_list(cls, numbers):
        """Creates a ``Transformation`` from a list of 16 numbers.

        Note:
            Since the transformation matrix follows the row-major order, the
            translational components must be at the list's indices 3, 7, 11.

        Args:
            numbers (:obj:`list` of :obj:`float`)

        Example:
            >>> numbers = [1, 0, 0, 3, 0, 1, 0, 4, 0, 0, 1, 5, 0, 0, 0, 1]
            >>> T = Transformation.from_list(numbers)
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
            >>> f1 = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
            >>> T = Transformation.from_frame(f1)
            >>> f2 = Frame.from_transformation(T)
            >>> f1 == f2
            True
        """
        T = cls()
        T.matrix = matrix_from_frame(frame)
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
            >>> f1 = Frame([2, 2, 2], [0.12, 0.58, 0.81], [-0.80, 0.53, -0.26])
            >>> f2 = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
            >>> T = Transformation.from_frame_to_frame(f1, f2)
            >>> f1.transform(T)
            >>> f1 == f2
            True
        """
        T1 = matrix_from_frame(frame_from)
        T2 = matrix_from_frame(frame_to)

        return cls(multiply_matrices(T2, inverse(T1)))

    def inverse(self):
        """Returns the inverse transformation.

        Example:
            >>> f = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
            >>> T = Transformation.from_frame(f)
            >>> I = Transformation() # identity matrix
            >>> I == T * T.inverse()
            True
        """
        cls = type(self)
        return cls(inverse(self.matrix))

    def decompose(self):
        """Decomposes the ``Transformation`` into ``Scale``, ``Shear``, \
            ``Rotation``, ``Translation`` and ``Perspective``.

        Example:
            >>> trans1 = [1, 2, 3]
            >>> angle1 = [-2.142, 1.141, -0.142]
            >>> scale1 = [0.123, 2, 0.5]
            >>> T1 = Translation(trans1)
            >>> R1 = Rotation.from_euler_angles(angle1)
            >>> S1 = Scale(scale1)
            >>> M = (T1 * R1) * S1
            >>> Sc, Sh, R, T, P = M.decompose()
            >>> S1 == Sc
            True
            >>> R1 == R
            True
            >>> T1 == T
            True
        """
        sc, sh, a, t, p = decompose_matrix(self.matrix)

        Sc = Scale(sc)
        Sh = Shear.from_entries(sh)
        R = Rotation.from_euler_angles(a, static=True, axes='xyz')
        T = Translation(t)
        P = Projection.from_entries(p)
        return Sc, Sh, R, T, P

    @property
    def rotation(self):
        """Returns the ``Rotation`` component from the ``Transformation``.
        """
        Sc, Sh, R, T, P = self.decompose()
        return R

    @property
    def translation(self):
        """Returns the 3 values of translation from the ``Transformation``.
        """
        return translation_from_matrix(self.matrix)

    @property
    def basis_vectors(self):
        """Returns the basis vectors from the ``Rotation`` component of the
            ``Transformation``.
        """
        sc, sh, a, t, p = decompose_matrix(self.matrix)
        R = matrix_from_euler_angles(a, static=True, axes='xyz')
        return basis_vectors_from_matrix(R)

    def transform_point(self, point):
        """Transforms a point.

        Args:
            point (:obj:`list` of :obj:`float`)

        Example:
            >>> f = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
            >>> T = Transformation.from_frame(f)
            >>> q = T.transform_point([0,0,0])
            >>> allclose(f.point, q)
            True

        Returns:
            (:obj:`list` of :obj:`float`): The transformed point.
        """

        ph = list(point) + [1.]  # make homogeneous coordinates
        pht = multiply_matrix_vector(self.matrix, ph)
        return pht[:3]

    def transform_points(self, points):
        """Transforms a list of points.

        Args:
            points (:obj:`list` of :obj:`list` of :obj:`float`): A list of
                points.

        Example:
            >>> f1 = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
            >>> T = Transformation.from_frame(f1)
            >>> pts = [[1.0, 1.0, 1.0], [1.68, 1.68, 1.27], [0.33, 1.73, 0.85]]
            >>> pts_ = T.transform_points(pts)

        Returns:
            (:obj:`list` of :obj:`list` of :obj:`float`): The transformed \
                points.
        """
        P = homogenize(points)
        Pt = map(list, zip(*P))  # transpose matrix
        Pt_ = multiply_matrices(self.matrix, Pt)
        return map(list, zip(*Pt_[:3]))  # cutoff 1 and transpose again

    @property
    def list(self):
        """Flattens the ``Transformation`` into a list of numbers.
        """
        return [a for c in self.matrix for a in c]

    def concatenate(self, other):
        """Concatenate two transformations into one ``Transformation``.

        Note:
            Rz * Ry * Rx means that Rx is first transformation, Ry second, and
            Rz third.
        """
        cls = type(self)
        if not isinstance(other, cls):
            return Transformation(multiply_matrices(self.matrix, other.matrix))
        else:
            return cls(multiply_matrices(self.matrix, other.matrix))

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
        s += "\n"
        return s


class Rotation(Transformation):
    """The ``Rotation`` represents a 4x4 rotation matrix and is based on \
        ``Transformation``.

    The class contains methods for converting rotation matrices to axis-angle
    representations, Euler angles, quaternion and basis vectors.

    Example:
        >>> f1 = Frame([0, 0, 0], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
        >>> R = Rotation.from_frame(f1)
        >>> args = False, 'xyz'
        >>> alpha, beta, gamma = R.euler_angles(*args)
        >>> xaxis, yaxis, zaxis = [1, 0, 0], [0, 1, 0], [0, 0, 1]
        >>> Rx = Rotation.from_axis_and_angle(xaxis, alpha)
        >>> Ry = Rotation.from_axis_and_angle(yaxis, beta)
        >>> Rz = Rotation.from_axis_and_angle(zaxis, gamma)
        >>> f2 = Frame.worldXY()
        >>> f2.transform(Rx * Ry * Rz)
        >>> f1 == f2
        True

    """

    @classmethod
    def from_basis_vectors(cls, xaxis, yaxis):
        """Creates a ``Rotation`` from basis vectors (= orthonormal vectors).

        Args:
            xaxis (:obj:`list` oof :obj:`float`): The x-axis of the frame.
            yaxis (:obj:`list` oof :obj:`float`): The y-axis of the frame.

        Example:
            >>> xaxis = [0.68, 0.68, 0.27]
            >>> yaxis = [-0.67, 0.73, -0.15]
            >>> R = Rotation.from_basis_vectors(xaxis, yaxis)

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
            quaternion (:obj:`list` of :obj:`float`): Four numbers that
                represents the four coefficient values of a quaternion.

        Example:
            >>> q1 = [0.945, -0.021, -0.125, 0.303]
            >>> R = Rotation.from_quaternion(q1)
            >>> q2 = R.quaternion
            >>> allclose(q1, q2, tol=1e-3)
            True
        """
        R = matrix_from_quaternion(quaternion)
        return cls(R)

    @classmethod
    def from_axis_angle_vector(cls, axis_angle_vector, point=[0, 0, 0]):
        """Calculates a ``Rotation`` from an axis-angle vector.

        Args:
            axis_angle_vector (:obj:`list` of :obj:`float`): Three numbers
                that represent the axis of rotation and angle of rotation
                through the vector's magnitude.
            point (:obj:`list` of :obj:`float`, optional): A point to
                perform a rotation around an origin other than [0, 0, 0].

        Example:
            >>> aav1 = [-0.043, -0.254, 0.617]
            >>> R = Rotation.from_axis_angle_vector(aav1)
            >>> aav2 = R.axis_angle_vector
            >>> allclose(aav1, aav2)
            True
        """

        axis_angle_vector = list(axis_angle_vector)
        angle = length_vector(axis_angle_vector)
        return cls.from_axis_and_angle(axis_angle_vector, angle, point)

    @classmethod
    def from_axis_and_angle(cls, axis, angle, point=[0, 0, 0]):
        """Calculates a ``Rotation`` from a rotation axis and an angle and \
            an optional point of rotation.

        Note:
            The rotation is based on the right hand rule, i.e. anti-clockwise
            if the axis of rotation points towards the observer.

        Args:
            axis (:obj:`list` of :obj:`float`): Three numbers that represent
                the axis of rotation
            angle (:obj:`float`): The rotation angle in radians.
            point (:obj:`list` of :obj:`float`, optional): A point to
                perform a rotation around an origin other than [0, 0, 0].

        Example:
            >>> axis1 = normalize_vector([-0.043, -0.254, 0.617])
            >>> angle1 = 0.1
            >>> R = Rotation.from_axis_and_angle(axis1, angle1)
            >>> axis2, angle2 = R.axis_and_angle
            >>> allclose(axis1, axis2)
            True
            >>> allclose([angle1], [angle2])
            True
        """
        M = matrix_from_axis_and_angle(axis, angle, point)
        return cls(M)

    @classmethod
    def from_euler_angles(cls, euler_angles, static=True, axes='xyz'):
        """Calculates a ``Rotation`` from Euler angles.

        In 3D space any orientation can be achieved by composing three
        elemental rotations, rotations about the axes (x,y,z) of a coordinate
        system. A triple of Euler angles can be interpreted in 24 ways, which
        depends on if the rotations are applied to a static (extrinsic) or
        rotating (intrinsic) frame and the order of axes.

        Args:
            euler_angles(:obj:`list` of :obj:`float`): Three numbers that
                represent the angles of rotations about the defined axes.
            static(:obj:`bool`, optional): If true the rotations are applied to
                a static frame. If not, to a rotational. Defaults to true.
            axes(:obj:`str`, optional): A 3 character string specifying order
                of the axes. Defaults to 'xyz'.

        Example:
            >>> ea1 = 1.4, 0.5, 2.3
            >>> args = False, 'xyz'
            >>> R1 = Rotation.from_euler_angles(ea1, *args)
            >>> ea2 = R1.euler_angles(*args)
            >>> allclose(ea1, ea2)
            True
            >>> alpha, beta, gamma = ea1
            >>> xaxis, yaxis, zaxis = [1, 0, 0], [0, 1, 0], [0, 0, 1]
            >>> Rx = Rotation.from_axis_and_angle(xaxis, alpha)
            >>> Ry = Rotation.from_axis_and_angle(yaxis, beta)
            >>> Rz = Rotation.from_axis_and_angle(zaxis, gamma)
            >>> R2 = Rx * Ry * Rz
            >>> R1 == R2
            True
        """

        M = matrix_from_euler_angles(euler_angles, static, axes)
        return Rotation(M)

    @property
    def quaternion(self):
        """Returns the 4 quaternion coefficients from the ``Rotation``.

        Example:
            >>> q1 = [0.945, -0.021, -0.125, 0.303]
            >>> R = Rotation.from_quaternion(q1)
            >>> q2 = R.quaternion
            >>> allclose(q1, q2, tol=1e-3)
            True
        """
        return quaternion_from_matrix(self.matrix)

    @property
    def axis_and_angle(self):
        """Returns the axis and the angle of the ``Rotation``.

        Example:
            >>> axis1 = normalize_vector([-0.043, -0.254, 0.617])
            >>> angle1 = 0.1
            >>> R = Rotation.from_axis_and_angle(axis1, angle1)
            >>> axis2, angle2 = R.axis_and_angle
            >>> allclose(axis1, axis2)
            True
            >>> allclose([angle1], [angle2])
            True
        """
        return axis_and_angle_from_matrix(self.matrix)

    @property
    def axis_angle_vector(self):
        """Returns the axis-angle vector of the ``Rotation``.

        Returns:
            (:obj:`list` of :obj:`float`): Three numbers that represent the \
                axis of rotation and angle of rotation through the vector's \
                magnitude.

        Example:
            >>> aav1 = [-0.043, -0.254, 0.617]
            >>> R = Rotation.from_axis_angle_vector(aav1)
            >>> aav2 = R.axis_angle_vector
            >>> allclose(aav1, aav2)
            True
        """
        axis, angle = self.axis_and_angle
        return scale_vector(axis, angle)

    def euler_angles(self, static=True, axes='xyz'):
        """Returns Euler angles from the ``Rotation`` according to specified \
            axis sequence and rotation type.

        Args:
            static(:obj:`bool`, optional): If true the rotations are applied to
                a static frame. If not, to a rotational. Defaults to True.
            axes(:obj:`str`, optional): A 3 character string specifying the
                order of the axes. Defaults to 'xyz'.

        Returns:
            (:obj:`list` of :obj:`float`): The 3 Euler angles.

        Example:
            >>> ea1 = 1.4, 0.5, 2.3
            >>> args = False, 'xyz'
            >>> R1 = Rotation.from_euler_angles(ea1, *args)
            >>> ea2 = R1.euler_angles(*args)
            >>> allclose(ea1, ea2)
            True
        """

        return euler_angles_from_matrix(self.matrix, static, axes)

    @property
    def basis_vectors(self):
        """Returns the basis vectors of the ``Rotation``.
        """
        return basis_vectors_from_matrix(self.matrix)


class Translation(Transformation):
    """Creates a translation transformation.

    Args:
        translation (:obj:`list` of :obj:`float`): a list of 3 numbers
            defining the translation in x, y, and z.

    Example:
        >>> T = Translation([1, 2, 3])
    """

    def __init__(self, translation):
        self.matrix = matrix_from_translation(translation)


class Scale(Transformation):
    """Creates a scaling transformation.

    Args:
        scale_factors (:obj:`list` of :obj:`float`): a list of 3 numbers
            defining the scaling factors in x, y, and z respectively.

    Example:
        >>> S = Scale([1, 2, 3])
    """

    def __init__(self, scale_factors):
        self.matrix = matrix_from_scale_factors(scale_factors)


class Reflection(Transformation):
    """Creates a ``Reflection`` that mirrors points at a plane, defined by
        point and normal vector.

    Args:
        point (:obj:`list` of :obj:`float`): The point of the mirror plane.
        normal (:obj:`list` of :obj:`float`): The normal of the mirror plane.

    Example:
        >>> point = [1, 1, 1]
        >>> normal = [0, 0, 1]
        >>> R = Reflection(point, normal)

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
        """Creates a ``Reflection`` that mirrors at the ``Frame``.

        Args:
            frame(:class:`Frame`)
        """
        return cls(frame.point, frame.normal)


class Projection(Transformation):

    @classmethod
    def orthogonal(cls, point, normal):
        """Returns an orthogonal ``Projection`` to project onto a plane \
            defined by point and normal.

        Args:
            point(:obj:`list` of :obj:`float`)
            normal(:obj:`list` of :obj:`float`)

        Example:
            >>> point = [0, 0, 0]
            >>> normal = [0, 0, 1]
            >>> P = Projection.ortogonal(point, normal)
        """
        T = cls()
        normal = normalize_vector(normal)

        for j in range(3):
            for i in range(3):
                T[i, j] -= normal[i] * normal[j]  # outer_product

        T[0, 3], T[1, 3], T[2, 3] = scale_vector(
            normal, dot_vectors(point, normal))
        return T

    @classmethod
    def parallel(cls, point, normal, direction):
        """Returns an parallel ``Projection`` to project onto a plane defined \
            by point, normal and direction.

        Args:
            point(:obj:`list` of :obj:`float`)
            normal(:obj:`list` of :obj:`float`)
            direction(:obj:`list` of :obj:`float`)

        Example:
            >>> point = [0, 0, 0]
            >>> normal = [0, 0, 1]
            >>> direction = [1, 1, 0]
            >>> P = Projection.parallel(point, normal, direction)
        """
        T = cls()
        normal = normalize_vector(normal)

        scale = dot_vectors(direction, normal)
        for j in range(3):
            for i in range(3):
                T[i, j] -= direction[i] * normal[j] / scale

        T[0, 3], T[1, 3], T[2, 3] = scale_vector(
            direction, dot_vectors(point, normal) / scale)
        return T

    @classmethod
    def perspective(cls, point, normal, perspective):
        """Returns an perspective ``Projection`` to project onto a plane \
            defined by point, normal and perspective.

        Args:
            point(:obj:`list` of :obj:`float`)
            normal(:obj:`list` of :obj:`float`)
            perspective(:obj:`list` of :obj:`float`)

        Example:
            >>> point = [0, 0, 0]
            >>> normal = [0, 0, 1]
            >>> perspective = [1, 1, 0]
            >>> P = Projection.perspective(point, normal, perspective)
        """
        T = cls()
        normal = normalize_vector(normal)

        T[0, 0] = T[1, 1] = T[2, 2] = dot_vectors(
            subtract_vectors(perspective, point), normal)

        for j in range(3):
            for i in range(3):
                T[i, j] -= perspective[i] * normal[j]

        T[0, 3], T[1, 3], T[2, 3] = scale_vector(
            perspective, dot_vectors(point, normal))
        for i in range(3):
            T[3, i] -= normal[i]
        T[3, 3] = dot_vectors(perspective, normal)
        return T

    @classmethod
    def from_entries(cls, perspective_entries):
        """Constructs a perspective transformation by the perspective entries \
            of a matrix.

        Args:
            perspective_entries(:obj:`list` of :obj:`float`): The 4 perspective
                entries of a matrix.
        """
        M = matrix_from_perspective_entries(perspective_entries)
        return cls(M)


class Shear(Transformation):
    """Constructs a ``Shear`` transformation by an angle along the \
        direction vector on the shear plane (defined by point and normal).

    A point P is transformed by the shear matrix into P" such that
    the vector P-P" is parallel to the direction vector and its extent is
    given by the angle of P-P'-P", where P' is the orthogonal projection
    of P onto the shear plane (defined by point and normal).

    Args:
        angle (:obj:`float`): The angle in radians.
        direction (:obj:`list` of :obj:`float`): The direction vector as
            list of 3 numbers. It must be orthogonal to the normal vector.
        point (:obj:`list` of :obj:`float`): The point of the shear plane
            as list of 3 numbers.
        normal (:obj:`list` of :obj:`float`): The normal of the shear plane
            as list of 3 numbers.

    Raises:
        ValueError: If direction and normal are not orthogonal.

    Example:
        >>> angle = 0.1
        >>> direction = [0.1, 0.2, 0.3]
        >>> point = [4, 3, 1]
        >>> normal = cross_vectors(direction, [1, 0.3, -0.1])
        >>> S = Shear(angle, direction, point, normal)
    """

    def __init__(self, angle=0., direction=[1, 0, 0],
                 point=[1, 1, 1], normal=[0, 0, 1]):

        self.matrix = matrix_from_shear(angle, direction, point, normal)

    @classmethod
    def from_entries(cls, shear_entries):
        """Creates a ``Shear`` from the 3 factors for x-y, x-z, and y-z axes.

        Args:
            shear_factors (:obj:`list` of :obj:`float`): The 3 shear factors
                for x-y, x-z, and y-z axes.

        Example:
            >>> S = Shear.from_entries([1, 2, 3])
        """
        M = matrix_from_shear_entries(shear_entries)
        return cls.from_matrix(M)


if __name__ == "__main__":

    from compas_fab.fab.geometry.frame import Frame
    f1 = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
    T = Transformation.from_frame(f1)
    f2 = Frame.from_transformation(T)
    print(f1 == f2)

    f = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
    T = Transformation.from_frame(f)
    Tinv = T.inverse()
    I = Transformation()
    print(I == T * Tinv)

    f1 = Frame([2, 2, 2], [0.12, 0.58, 0.81], [-0.80, 0.53, -0.26])
    f2 = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
    T = Transformation.from_frame_to_frame(f1, f2)
    f1.transform(T)
    print(f1 == f2)

    trans1 = [1, 2, 3]
    angle1 = [-2.142, 1.141, -0.142]
    scale1 = [0.123, 2, 0.5]
    T = matrix_from_translation(trans1)
    R = matrix_from_euler_angles(angle1)
    S = matrix_from_scale_factors(scale1)
    M = multiply_matrices(multiply_matrices(T, R), S)
    # M = compose_matrix(scale1, None, angle1, trans1, None)
    scale2, shear2, angle2, trans2, persp2 = decompose_matrix(M)
    print(allclose(scale1, scale2))
    print(allclose(angle1, angle2))
    print(allclose(trans1, trans2))

    T1 = Translation(trans1)
    R1 = Rotation.from_euler_angles(angle1)
    S1 = Scale(scale1)
    M = (T1 * R1) * S1
    S2, Sh, R2, T2, P = M.decompose()
    print(S1 == S2)
    print(R1 == R2)
    print(T1 == T2)

    shear1 = [-0.41, -0.14, -0.35]
    persp1 = [0.3, 0.1, 0.1, 1]
    Sh1 = Shear.from_entries(shear1)
    S2, Sh, R2, T2, P = Sh1.decompose()
    # print("Sh", Sh)

    P1 = Projection.from_entries(persp1)
    S2, Sh, R2, T2, P = P1.decompose()
    # print("P", P)

    f = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
    T = Transformation.from_frame(f)
    q = T.transform_point([0, 0, 0])
    print(allclose(f.point, q))

    f1 = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
    T = Transformation.from_frame(f1)
    pts = [[1.0, 1.0, 1.0], [1.68, 1.68, 1.27], [0.33, 1.73, 0.85]]
    pts_ = T.transform_points(pts)

    xaxis = [0.68, 0.68, 0.27]
    yaxis = [-0.67, 0.73, -0.15]
    R = Rotation.from_basis_vectors(xaxis, yaxis)

    q1 = [0.945, -0.021, -0.125, 0.303]
    R = Rotation.from_quaternion(q1)
    q2 = R.quaternion
    print(allclose(q1, q2, tol=1e-3))

    aav1 = [-0.043, -0.254, 0.617]
    R = Rotation.from_axis_angle_vector(aav1)
    aav2 = R.axis_angle_vector
    print(allclose(aav1, aav2))

    axis1 = normalize_vector([-0.043, -0.254, 0.617])
    angle1 = 0.1
    R = Rotation.from_axis_and_angle(axis1, angle1)
    axis2, angle2 = R.axis_and_angle
    print(allclose(axis1, axis2))
    print(allclose([angle1], [angle2]))

    ea1 = 1.4, 0.5, 2.3
    args = False, 'xyz'
    R1 = Rotation.from_euler_angles(ea1, *args)
    ea2 = R1.euler_angles(*args)
    print(allclose(ea1, ea2))

    alpha, beta, gamma = ea1
    origin, xaxis, yaxis, zaxis = [0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]
    Rx = Rotation.from_axis_and_angle(xaxis, alpha)
    Ry = Rotation.from_axis_and_angle(yaxis, beta)
    Rz = Rotation.from_axis_and_angle(zaxis, gamma)
    R2 = Rx * Ry * Rz
    print(R1 == R2)

    f1 = Frame([0, 0, 0], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
    R = Rotation.from_frame(f1)
    args = False, 'xyz'
    alpha, beta, gamma = R.euler_angles(*args)
    xaxis, yaxis, zaxis = [1, 0, 0], [0, 1, 0], [0, 0, 1]
    Rx = Rotation.from_axis_and_angle(xaxis, alpha)
    Ry = Rotation.from_axis_and_angle(yaxis, beta)
    Rz = Rotation.from_axis_and_angle(zaxis, gamma)
    f2 = Frame.worldXY()
    f2.transform(Rx * Ry * Rz)
    print(f1 == f2)

    angle = 0.1
    direction = [0.1, 0.2, 0.3]
    point = [4, 3, 1]
    normal = cross_vectors(direction, [1, 0.3, -0.1])
    S = Shear(angle, direction, point, normal)
    print(S)
    S = Shear.from_entries([1, 2, 3])

    import inspect
    name, suffix, mode, mtype = inspect.getmoduleinfo(__file__)
