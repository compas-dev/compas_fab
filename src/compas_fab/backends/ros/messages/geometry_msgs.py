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
        quaternion = [self.orientation.w, self.orientation.x,
                      self.orientation.y, self.orientation.z]
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

    Examples
    --------
    >>> wrench = compas_fab.robots.Wrench([0, 0, -98], [0, 0, 0])
    >>> ros_wrench = Wrench.from_wrench(wrench)
    >>> ros_wrench.msg
    {'force': {'x': 0.0, 'y': 0.0, 'z': -98.0}, 'torque': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
    >>> ros_wrench.wrench
    Wrench(Vector(0.000, 0.000, -98.000), Vector(0.000, 0.000, 0.000))
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


class Inertia(ROSmsg):
    """http://docs.ros.org/api/geometry_msgs/html/msg/Inertia.html

    Examples
    --------
    >>> inertia = compas_fab.robots.Inertia([[0] * 3] * 3, 1., [0.1, 3.1, 4.4])
    >>> ros_inertia = Inertia.from_inertia(inertia)
    >>> ros_inertia.msg
    {'m': 1.0, 'com': {'x': 0.1, 'y': 3.1, 'z': 4.4}, 'ixx': 0.0, 'ixy': 0.0, 'ixz': 0.0, 'iyy': 0.0, 'iyz': 0.0, 'izz': 0.0}
    >>> ros_inertia.inertia
    Inertia([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], 1.0, Point(0.100, 3.100, 4.400))
    """

    def __init__(self, m=0.0, com=None, ixx=0., ixy=0., ixz=0., iyy=0., iyz=0., izz=0.):
        self.m = float(m)  # Mass [kg]
        self.com = com or Vector3()  # Center of mass [m]
        self.ixx = float(ixx)
        self.ixy = float(ixy)
        self.ixz = float(ixz)
        self.iyy = float(iyy)
        self.iyz = float(iyz)
        self.izz = float(izz)

    @classmethod
    def from_msg(cls, msg):
        com = Vector3.from_msg(msg['com'])
        return cls(msg['m'], com, msg['ixx'], msg['ixy'], msg['ixz'], msg['iyy'], msg['iyz'], msg['izz'])

    @classmethod
    def from_inertia(cls, inertia):
        m = inertia.mass
        com = Vector3(*list(inertia.center_of_mass))
        ixx, ixy, ixz = inertia.inertia_tensor[0]
        ixy, iyy, iyz = inertia.inertia_tensor[1]
        izz = inertia.inertia_tensor[2][2]
        return cls(m, com, ixx, ixy, ixz, iyy, iyz, izz)

    @property
    def inertia(self):
        inertia_tensor = [[self.ixx, self.ixy, self.ixz],
                          [self.ixy, self.iyy, self.iyz],
                          [self.ixz, self.iyz, self.izz]]
        return compas_fab.robots.Inertia(inertia_tensor, self.m, [self.com.x, self.com.y, self.com.z])


if __name__ == "__main__":
    import doctest
    doctest.testmod()
