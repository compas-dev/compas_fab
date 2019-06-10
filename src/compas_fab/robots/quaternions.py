
from compas.geometry.transformations import quaternion_from_matrix
from compas.geometry.transformations import matrix_from_quaternion
from compas.geometry.transformations import euler_angles_from_matrix
from compas.geometry.transformations import matrix_from_euler_angles
import math

# sources:
# https://github.com/matthew-brett/transforms3d/blob/master/transforms3d/quaternions.py
# http://mathworld.wolfram.com/Quaternion.html

"""
This module contains functions that operate on and/or return quaternions.

Default convention to represent a quaternion in this module is a tuple of four real values [qw,qx,qy,qz].
The first value qw is the scalar (real) part, and qx,qy,qz form the vector (complex, imaginary) part, so that:
q = qw + qx*i + qy*j + qz*k,
where i,j,k are basis components.

Quaternion algebra basics:
i*i = j*j = k*k = i*j*k = -1
i*j = k    j*i = -k
j*k = i    k*j = -i
k*i = j    i*k = -j

Quaternion as rotation:
A rotation through an angle theta around an axis defined by a euclidean unit vector u = ux*i + uy*j + uz*k
can be represented as a quaternion:
q = cos(theta/2) + sin(theta/2) * [ux*i + uy*j + uz*k]
i.e.:
qw = cos(theta/2)
qx = sin(theta/2) * ux
qy = sin(theta/2) * uy
qz = sin(theta/2) * uz

For a quaternion to represent a rotation or orientation, it must be unit-length.

"""

# ----------------------------------------------------------------------
ATOL = 1e-16  # absolute tolerance
RTOL = 1e-3  # relative tolerance


def isclose(a, b, atol=ATOL, rtol=RTOL):
    # https://docs.scipy.org/doc/numpy/reference/generated/numpy.isclose.html#numpy.isclose
    # absolute(a - b) <= (atol + rtol * absolute(b))
    return abs(a - b) <= (atol + rtol * abs(b))

# ----------------------------------------------------------------------


def q_norm(q):
    # http://mathworld.wolfram.com/QuaternionNorm.html
    """
    Returns the length (euclidean norm) of a quaternion.
    """
    return math.sqrt(sum([x*x for x in q]))


def q_unitize(q):
    """
    Returns the a quaternion of length 1.0, or False if failed.
    """
    n = q_norm(q)
    if isclose(n, 0.0):
        #raise ValueError("The given quaternion has zero length")
        print("The given quaternion has zero length")
        return False
    else:
        return [x/n for x in q]


def q_is_unit(q):
    """
    Returns True if the quaternion is unit-length, and False if otherwise.
    """
    n = q_norm(q)
    return isclose(n, 1.0)


def q_multiply(r, q):
    # http://mathworld.wolfram.com/Quaternion.html
    """
    Returns a quaternion p = r * q.
    This can be interpreted as applying rotation r to an orientation q, provided both r and q are unit-length.
    The result is also unit-length.
    Multiplication of quaternions is not commutative!
    """
    rw, rx, ry, rz = r
    qw, qx, qy, qz = q
    pw = rw*qw - rx*qx - ry*qy - rz*qz
    px = rw*qx + rx*qw + ry*qz - rz*qy
    py = rw*qy - rx*qz + ry*qw + rz*qx
    pz = rw*qz + rx*qy - ry*qx + rz*qw
    return [pw, px, py, pz]


def q_canonic(q):
    """
    Returns a quaternion in a canonic form.
    Canonic form means the scalar component is a non-negative number.
    """
    if q[0] < 0.0:
        return [-x for x in q]
    else:
        return [x for x in q]


def q_conjugate(q):
    # http://mathworld.wolfram.com/QuaternionConjugate.html
    """
    Returns a conjucate of a given quaternion.
    """
    return [q[0], -q[1], -q[2], -q[3]]


# ----------------------------------------------------------------------
def q_from_matrix(m):
    # already in compas/geometry/transformations/matrices.py
    return quaternion_from_matrix(m)


def matrix_from_q(q):
    # already in compas/geometry/transformations/matrices.py
    return matrix_from_quaternion(q)

# ----------------------------------------------------------------------


def quaternion_from_euler_angles(e, static=True, axes='xyz'):
    m = matrix_from_euler_angles(e, static=True, axes='xyz')
    q = quaternion_from_matrix(m)
    return q


def euler_angles_from_quaternion(q, static=True, axes='xyz'):
    m = matrix_from_quaternion(q)
    e = euler_angles_from_matrix(m, static, axes)
    return e

# ----------------------------------------------------------------------


def quaternion_from_axis_angle(axis, angle):
    raise NotImplementedError


def axis_angle_from_quaternion(q):
    raise NotImplementedError

# ----------------------------------------------------------------------


class Quaternion(object):
    def __init__(self, q, convention='wxyz'):

        # TODO: add checks for len(q)==4, value type

        if convention == 'wxyz':
            pass
        elif convention == 'xyzw':
            q = [q[3], q[0], q[1], q[2]]
        else:
            raise ValueError("Invalid quaternion convention: expected 'wxyz' (default) or 'xyzw'.")

        self.q = q
        self.qw = self.q[0]
        self.qx = self.q[1]
        self.qy = self.q[2]
        self.qz = self.q[3]

    def __str__(self):
        return "Quaternion = %s" % self.q

    def __mul__(self, other_q):
        """
        Reminder: for a quaternion multiplication to represent a composition of rotations, both quaternions must be unit-length!
        The result is also unit-length.

        Example:
        -------
        R = Quaternion([-6,7,-8,9]).unitized()
        Q = Quaternion([1,-2,3,-4]).unitized()
        P = R * Q

        print(P)
        >>> Quaternion = [0.8186238009832305, 0.28892604740584604, -0.19261736493723072, 0.4574662417259229]
        """
        p = q_multiply(self.q, other_q.q)
        return Quaternion(p)

    def xyzw(self):
        """
        Returns the quaternion as a tuple in the xyzw convention
        """
        return [self.qx, self.qy, self.qz, self.qw]

    def conjugate(self):
        return Quaternion(q_conjugate(self.q))

    def unitize(self):
        qu = q_unitize(self.q)
        if qu:
            self.q = qu
            return True
        else:
            return False

    def unitized(self):
        qu = q_unitize(self.q)
        if qu:
            return Quaternion(qu)
        else:
            return None

    def canonize(self):
        qc = q_canonic(self.q)
        self.q = qc
        return True

    def canonized(self):
        qc = q_canonic(self.q)
        return Quaternion(qc)

    @property
    def norm(self):
        return q_norm(self.q)


if __name__ == "__main__":
    import doctest
    doctest.testmod(globs=globals())
