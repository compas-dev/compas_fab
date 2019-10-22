from __future__ import absolute_import

from compas.geometry import Frame

import compas_fab.robots

from .std_msgs import Header
from .std_msgs import ROSmsg


class Point(ROSmsg):
    """http://docs.ros.org/api/geometry_msgs/html/msg/Point.html
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
    """http://docs.ros.org/api/geometry_msgs/html/msg/Quaternion.html
    """

    def __init__(self, x=0., y=0., z=0., w=1.):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    @classmethod
    def from_frame(cls, frame):
        qw, qx, qy, qz = frame.quaternion
        return cls(qx, qy, qz, qw)


class Pose(ROSmsg):
    """http://docs.ros.org/api/geometry_msgs/html/msg/Pose.html
    """

    def __init__(self, position=None, orientation=None):
        self.position = position if position else Point(0, 0, 0)
        self.orientation = orientation if orientation else Quaternion(0, 0, 0, 1)

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
    """http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
    """

    def __init__(self, header=None, pose=None):
        self.header = header or Header()
        self.pose = pose or Pose()

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

    @classmethod
    def from_msg(cls, msg):
        x, y, z = msg['x'], msg['y'], msg['z']
        return cls(x, y, z)


class Transform(ROSmsg):
    """http://docs.ros.org/api/geometry_msgs/html/msg/Transform.html
    """

    def __init__(self, translation=None, rotation=None):
        self.translation = translation or Vector3()
        self.rotation = rotation or Quaternion()


class Twist(ROSmsg):
    """http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
    """

    def __init__(self, linear=None, angular=None):
        self.linear = linear or Vector3()
        self.angular = angular or Vector3()


class Wrench(ROSmsg):
    """http://docs.ros.org/api/geometry_msgs/html/msg/Wrench.html

    This represents force in free space, separated into its linear and angular parts.
    """

    def __init__(self, force=None, torque=None):
        self.force = force or Vector3()
        self.torque = torque or Vector3()

    @classmethod
    def from_msg(cls, msg):
        force = Vector3.from_msg(msg['force'])
        torque = Vector3.from_msg(msg['torque'])
        return cls(force, torque)

    @classmethod
    def from_wrench(cls, wrench):
        force = wrench.force
        torque = wrench.torque
        return cls(Vector3(*list(force)), Vector3(*list(torque)))

    @property
    def wrench(self):
        force = [self.force.x, self.force.y, self.force.z]
        torque = [self.torque.x, self.torque.y, self.torque.z]
        return compas_fab.robots.Wrench(force, torque)


class WrenchStamped(ROSmsg):
    """http://docs.ros.org/api/geometry_msgs/html/msg/WrenchStamped.html

    A wrench with reference coordinate frame and timestamp.
    """

    def __init__(self, header=None, wrench=None):
        self.header = header or Header()
        self.wrench = wrench or Wrench()

    @classmethod
    def from_msg(cls, msg):
        header = Header.from_msg(msg['header'])
        wrench = Wrench.from_msg(msg['wrench'])
        return cls(header, wrench)
