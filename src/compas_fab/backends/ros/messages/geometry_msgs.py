from __future__ import absolute_import

from .std_msgs import ROSmsg
from .std_msgs import Header
from .std_msgs import Time

from compas.geometry import Frame


class Point(ROSmsg):
    """http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Point.html
    """
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    @classmethod
    def from_msg(cls, msg):
        x, y, z = msg['x'], msg['y'], msg['z']
        return cls(x, y, z)

class Quaternion(ROSmsg):
    """http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Quaternion.html
    """
    def __init__(self, x=0. ,y=0., z=0., w=1.):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    @classmethod
    def from_frame(cls, frame):
        qw, qx, qy, qz = frame.quaternion
        return cls(qx, qy, qz, qw)

class Pose(ROSmsg):
    """http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html
    """
    def __init__(self, position=None, orientation=None):
        self.position = position if position else Point(0,0,0)
        self.orientation = orientation if orientation else Quaternion(0,0,0,1)

    @classmethod
    def from_frame(cls, frame):
        point = frame.point
        qw, qx, qy, qz = frame.quaternion
        return cls(Point(*list(point)), Quaternion(qx, qy, qz, qw))

    @property
    def frame(self):
        point = [self.position.x, self.position.y, self.position.z]
        quaternion = [self.orientation.w, self.orientation.x, self.orientation.y, self.orientation.z]
        return Frame.from_quaternion(quaternion, point=point)

    @classmethod
    def from_msg(cls, msg):
        position = Point.from_msg(msg['position'])
        orientation = Quaternion.from_msg(msg['orientation'])
        return cls(position, orientation)

class PoseStamped(ROSmsg):
    """http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html
    """
    def __init__(self, header=Header(), pose=Pose()):
        self.header = header
        self.pose = pose

    @classmethod
    def from_msg(cls, msg):
        header = Header.from_msg(msg['header'])
        pose = Pose.from_msg(msg['pose'])
        return cls(header, pose)

class Vector3(ROSmsg):
    """http://docs.ros.org/api/geometry_msgs/html/msg/Vector3.html
    """
    def __init__(self, x=0., y=0., z=0.):
        self.x = x
        self.y = y
        self.z = z

class Transform(ROSmsg):
    """http://docs.ros.org/api/geometry_msgs/html/msg/Transform.html
    """
    def __init__(self, translation=Vector3(), rotation=Quaternion()):
        self.translation = translation
        self.rotation = rotation

class TransformStamped(ROSmsg):
    """http://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html
    """
    def __init__(self, header=Header(), child_frame_id='', transform=Transform()):
        self.header = header
        self.child_frame_id = child_frame_id
        self.transform = transform

class Twist(ROSmsg):
    """http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
    """
    def __init__(self, linear=Vector3(), angular=Vector3()):
        self.linear = linear
        self.angular = angular
