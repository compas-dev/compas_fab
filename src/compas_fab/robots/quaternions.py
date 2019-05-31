
import compas.geometry.transformations as cgt

#sources:
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


#----------------------------------------------------------------------
def q_norm(q):
    #http://mathworld.wolfram.com/QuaternionNorm.html
    raise NotImplementedError

def q_unitize(q):
    raise NotImplementedError

def q_is_unit(q):
    raise NotImplementedError

def q_multiply(q1,q0):
    raise NotImplementedError

def q_canonic(q):
    raise NotImplementedError

def q_conjugate(q):
    raise NotImplementedError

#----------------------------------------------------------------------
def quaternion_from_matrix(m):
    # already in compas/geometry/transformations/matrices.py
    return cgt.quaternion_from_matrix(m)

def matrix_from_quaternion(q):
    # already in compas/geometry/transformations/matrices.py
    return cgt.matrix_from_quaternion

#----------------------------------------------------------------------
def quaternion_from_euler_angles(e, static=True, axes='xyz'):
    raise NotImplementedError

def euler_angles_from_quaternion(q, static=True, axes='xyz'):
    m = cgt.matrix_from_quaternion
    e = cgt.euler_angles_from_matrix(m, static, axes)
    return e

#----------------------------------------------------------------------
def quaternion_from_axis_angle(axis,angle):
    raise NotImplementedError

def axis_angle_from_quaternion(q):
    raise NotImplementedError

#----------------------------------------------------------------------
class Quaternion(object):
    def __init__(self, q, convention = 'wxyz'):

        if convention == 'wxyz':
            pass
        elif convention == 'xyzw':
            q = [q[3], q[0], q[1], q[2]]
        else:
            raise ValueError("Invalid quaternion convention: expected 'wxyz' (default) or 'xyzw'." )

        self.qw = q[0]
        self.qx = q[1]
        self.qy = q[2]
        self.qz = q[3]

    def q(self, convention = 'wxyz'):
        """
        Returns the quaternion as a tuple in the given convention
        """
        raise NotImplementedError



if __name__ == "__main__":
    import doctest
    doctest.testmod(globs=globals())
