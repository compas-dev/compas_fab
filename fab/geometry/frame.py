from __future__ import print_function
from compas.geometry.basic import cross_vectors
from compas.geometry.basic import normalize_vector
from compas_fab.fab.geometry.transformation import Transformation
from compas_fab.fab.geometry.transformation import Rotation
from compas_fab.fab.utilities.numbers import allclose

__author__ = ['Romana Rust <rust@arch.ethz.ch>', ]


class Frame():
    """The ``Frame`` consists of a point and and two orthonormal base vectors.

    It represents a plane in three dimensions with a defined origin and
    orientation.

    Args:
        point (:obj:`iterable` of :obj:`float`, optional): The origin of the
            frame. Defaults to [0, 0, 0].
        xaxis (:obj:`iterable` of :obj:`float`, optional): The x-axis of the
            frame. Defaults to [1, 0, 0].
        yaxis (:obj:`iterable` of :obj:`float`, optional): The y-axis of the
            frame. Defaults to [0, 1, 0].

    Examples:
        frame = Frame([0, 0, 0], [1, 0, 0], [0, 1, 0])
        frame = Frame.worldXY()
        frame = Frame.from_transformation(transformation)
        frame = Frame.from_rotation(rotation)
    """

    def __init__(self, point=[0, 0, 0], xaxis=[1, 0, 0], yaxis=[0, 1, 0]):
        self.point = [float(f) for f in list(point)]
        self.xaxis = list(normalize_vector(list(xaxis)))
        self.yaxis = list(normalize_vector(list(yaxis)))
        self.yaxis = list(cross_vectors(self.zaxis, self.xaxis))  # correction

    def copy(self):
        """Returns a copy of the frame.
        """
        cls = type(self)
        return cls(self.point[:], self.xaxis[:], self.yaxis[:])

    @classmethod
    def worldXY(cls):
        """Returns the world XY frame.
        """
        return cls([0, 0, 0], [1, 0, 0], [0, 1, 0])

    @classmethod
    def worldZX(cls):
        """Returns the world ZX frame.
        """
        return cls([0, 0, 0], [0, 0, 1], [1, 0, 0])

    @classmethod
    def worldYZ(cls):
        """Returns the world YZ frame.
        """
        return cls([0, 0, 0], [0, 1, 0], [0, 0, 1])

    @classmethod
    def from_rotation(cls, rotation, point=[0, 0, 0]):
        """Calculates a frame from the passed rotation.

        Args:
            rotation (:class:`Rotation`): The rotation defines the orientation
                of the frame through the base vectors.
            point (:obj:`iterable` of :obj:`float`, optional): The point of the
                frame. Defaults to [0, 0, 0].

        Example:
            R = Rotation.from_axis_angle_vector([-0.040, -0.319, 0.757])
            f = Frame.from_rotation(R, point = [1, 1, 1])
        """
        # TODO: better method
        xaxis, yaxis = rotation.basis_vectors
        return cls(point, xaxis, yaxis)

    @classmethod
    def from_transformation(cls, transformation):
        """Calculates a frame from a transformation.

        Args:
            transformation (:class:`Transformation`): The transformation
                defines the orientation of the frame through the rotation and
                the point through the translation.

        Example:
            T = Transformation...
            f = Frame.from_transformation(T)
        """
        xaxis, yaxis = transformation.basis_vectors
        # TODO: better method
        point = transformation.translation().vector
        return cls(point, xaxis, yaxis)

    @classmethod
    def from_quaternion(cls, quaternion, point=[0, 0, 0]):
        """Calculates a frame from a rotation represented by quaternion
            coefficients.

        Args:
            quaternion (:obj:`iterable` of :obj:`float`): Four numbers that
                represents the four coefficient values of a quaternion.
            point (:obj:`iterable` of :obj:`float`, optional): The point of the
                frame. Defaults to [0, 0, 0].

        Example:
            q = [0.945, -0.021, -0.125, 0.303]
            frame = Frame.from_quaternion(q)
        """

        rotation = Rotation.from_quaternion(quaternion)
        frame = cls.from_rotation(rotation)
        frame.point = point
        return frame

    @classmethod
    def from_axis_angle_vector(cls, axis_angle_vector, point=[0, 0, 0]):
        """Calculates a frame from a rotation represented by an axis-angle
            vector.

        Args:
            axis_angle_vector (:obj:`iterable` of :obj:`float`): Three numbers
                that represent the axis of rotation and angle of rotation by
                its magnitude.
            point (:obj:`iterable` of :obj:`float`, optional): The point of the
                frame. Defaults to [0, 0, 0].

        Example:
            frame = Frame.from_axis_angle_vector([-0.043, -0.254, 0.617],
                point = [19, 16, 12])
        """
        rotation = Rotation.from_axis_angle_vector(axis_angle_vector)
        frame = cls.from_rotation(rotation)
        frame.point = point
        return frame

    @classmethod
    def from_euler_angles(cls, euler_angles, point=[0, 0, 0]):
        """Calculates a frame from a rotation represented by Euler angles.

        Args:
            euler_angles (:obj:`iterable` of :obj:`float`): Three numbers a, b,
                c that represent the angles of rotations about each of the
                coordinate axes x, y, z.
            point (:obj:`iterable` of :obj:`float`, optional): the point of the
                frame. Defaults to [0, 0, 0].

        Example:
            frame = Frame.from_euler_angles([1,2,3,4], point = [5,5,0])
        """
        rotation = Rotation.from_euler_angles(euler_angles)
        frame = cls.from_rotation(rotation)
        frame.point = point
        return frame

    @classmethod
    def from_points(cls, point, point_xaxis, point_xyplane):
        """Calculates a frame from 3 points.

        Args:
            point (:obj:`iterable` of :obj:`float`): The origin of the frame.
            point_xaxis (:obj:`iterable` of :obj:`float`): A point on the
                x-axis of the frame.
            point_xyplane (:obj:`iterable` of :obj:`float`): A point within the
                xy-plane of frame
        """
        xaxis = subtract_vectors(point_xaxis, point)
        xyvec = subtract_vectors(point_xyplane, point)
        yaxis = list(cross_vectors(cross_vectors(xaxis, xyvec), xaxis))
        return cls(point, xaxis, yaxis)

    @property
    def normal(self):
        """Returns the frame's z-axis (normal).
        """
        return cross_vectors(self.xaxis, self.yaxis)

    @property
    def zaxis(self):
        """Returns the frame's z-axis (normal).
        """
        return self.normal

    @property
    def quaternion(self):
        """Returns the 4 quaternion coefficients from the rotation given by the
            frame.
        """
        rotation = Rotation.from_basis_vectors(self.xaxis, self.yaxis)
        return rotation.quaternion

    @property
    def axis_angle_vector(self):
        """Returns the axis-angle vector from the rotation given by the frame.
        """
        rotation = Rotation.from_basis_vectors(self.xaxis, self.yaxis)
        return rotation.axis_angle_vector

    @property
    def euler_angles(self):
        """Returns the Euler angles from the rotation given by the frame.
        """
        R = Rotation.from_basis_vectors(self.xaxis, self.yaxis)
        return R.euler_angles

    @property
    def rotation(self):
        """Returns the frame's rotation in regards to world XY.
        """
        return Rotation.from_basis_vectors(self.xaxis, self.yaxis)

    def transform(self, transformation, copy=False):
        """Transforms the frame with the passed transformation.

        Args:
            transformation (:class:`Transformation`): The transformation used
                to transform the Frame.
            copy (:obj:`bool`, optional): If true, a copy of the frame will be
                made. Defaults to false.

        Returns:
            (:class:`Transformation`): The transformed frame.
        """

        T = transformation * Transformation.from_frame(self)
        point = T.translation().vector
        xaxis, yaxis = T.basis_vectors

        if copy:
            return Frame(point, xaxis, yaxis)
        else:
            self.point = point
            self.xaxis = xaxis
            self.yaxis = yaxis
            return self

    def __repr__(self):
        s = "[[%.4f, %.4f, %.4f], " % tuple(self.point)
        s += "[%.4f, %.4f, %.4f], " % tuple(self.xaxis)
        s += "[%.4f, %.4f, %.4f]]" % tuple(self.yaxis)
        return s

    def __eq__(self, other):
        raise NotImplementedError


if __name__ == '__main__':

    from compas.geometry import subtract_vectors

    frame = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
    pt = frame.point

    print(frame)
    print("--")
    print("xaxis", frame.xaxis)
    print("yaxis", frame.yaxis)
    print("quaternion", frame.quaternion)
    print("axis_angle_vector", frame.axis_angle_vector)
    print("--")
    print(Frame.from_quaternion(frame.quaternion, pt))
    print(Frame.from_axis_angle_vector(frame.axis_angle_vector, pt))

    q1 = [0.945, -0.021, -0.125, 0.303]
    frame = Frame.from_quaternion(q1, point=[19, 16, 12])
    q2 = frame.quaternion
    print(allclose(q1, q2, 1e-3))
    print(frame.axis_angle_vector)

    axis_angle_vector = [-0.040, -0.319, 0.757]
    R = Rotation.from_axis_angle_vector([-0.040, -0.319, 0.757])
    f = Frame.from_rotation(R, point=[1, 1, 1])

    axis_angle_vector = [-0.040, -0.319, 0.757]
    f = Frame.from_axis_angle_vector([-0.040, -0.319, 0.757], point=[1, 1, 1])

    """
    frame = Frame([0, 0, 0], [1, 0, 0], [0, 1, 0])
    frame = Frame.worldXY()
    frame = Frame.from_transformation(transformation)
    frame = Frame.from_rotation(rotation)
    # TODO: make example code
    """
    f1 = Frame([0, 0, 0], [0.12, 0.58, 0.81], [-0.80, 0.53, -0.26])
    print(f1)
