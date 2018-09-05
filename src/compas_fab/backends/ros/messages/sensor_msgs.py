from __future__ import absolute_import

from .std_msgs import ROSmsg
from .std_msgs import Header
from .std_msgs import Time

class JointState(ROSmsg):
    """http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/JointState.html
    """

    def __init__(self, header=Header(), name=[], position=[], velocity=[],
                 effort=[]):
        self.header = header
        self.name = name
        self.position = position
        self.velocity = velocity
        self.effort = effort

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

    def __init__(self, header=Header(), joint_names=[], transforms=[], twist=[],
                 wrench=[]):
        self.header = header
        self.joint_names = joint_names
        self.transforms = transforms
        self.twist = twist
        self.wrench = wrench
