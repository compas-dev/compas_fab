from __future__ import absolute_import

from .std_msgs import ROSmsg
from .std_msgs import Header


class JointState(ROSmsg):
    """http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/JointState.html
    """

    def __init__(self, header=None, name=None, position=None, velocity=None,
                 effort=None):
        self.header = header if header else Header()
        self.name = name if name else []
        self.position = position if position else []
        self.velocity = velocity if velocity else []
        self.effort = effort if effort else []

    @classmethod
    def from_name_and_position(cls, name, position):
        return cls(Header(), name, position, [], [])

    @classmethod
    def from_configuration(cls):
        pass

    @property
    def configuration(self):
        pass

    @classmethod
    def from_msg(cls, msg):
        header = Header.from_msg(msg['header'])
        name = msg['name']
        position = msg['position']
        velocity = msg['velocity']
        effort = msg['effort']
        return cls(header, name, position, velocity, effort)


class MultiDOFJointState(ROSmsg):
    """http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/MultiDOFJointState.html
    """

    def __init__(self, header=None, joint_names=None, transforms=None, twist=None,
                 wrench=None):
        self.header = header if header else Header()
        self.joint_names = joint_names if joint_names else []
        self.transforms = transforms if transforms else []
        self.twist = twist if twist else []
        self.wrench = wrench if wrench else []
