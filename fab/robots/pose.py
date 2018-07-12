from compas.geometry.objects.frame import Frame
from compas.geometry.transformations import matrix_from_quaternion
from compas.geometry.transformations import basis_vectors_from_matrix

class Pose(Frame):
    """Represents a robot pose.
    
    In principal the ``Pose`` is a wrapper object around the frame to derive 
    rosbridge messages therefrom.

    Examples:
        >>> f = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
        >>> p1 = Pose.from_frame(f)
        >>> msg = p1.msg
        >>> p2 = Pose.from_msg(msg)
        >>> print(p1 == p2)
    """

    @classmethod
    def from_frame(cls, frame):
        return cls(frame.point, frame.xaxis, frame.yaxis)

    @property
    def frame(self):
        return Frame(self.point, self.xaxis, self.yaxis)

    @classmethod
    def from_msg(cls, msg):
        point = [msg['position']['x'], msg['position']['y'], \
            msg['position']['z']]
        quaternion = [msg['orientation']['w'], msg['orientation']['x'], \
            msg['orientation']['y'], msg['orientation']['z']]
        R = matrix_from_quaternion(quaternion)
        xaxis, yaxis = basis_vectors_from_matrix(R)
        return cls(point, xaxis, yaxis)
    
    @property
    def msg(self):
        """Returns the pose as dictionary to use with rosbridge.

        http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html
        """
        pose = {}
        pose['position'] = {'x':self.point[0], 'y':self.point[1], 'z':self.point[2]}
        qw, qx, qy, qz = self.quaternion
        pose['orientation'] = {'x': qx, 'y': qy, 'z': qz, 'w': qw}
        return pose


if __name__ == '__main__':

    f = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
    p1 = Pose.from_frame(f)
    msg = p1.msg
    print(msg)
    p2 = Pose.from_msg(msg)
    print(p1 == p2)