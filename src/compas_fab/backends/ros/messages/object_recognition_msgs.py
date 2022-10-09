from __future__ import absolute_import

from .std_msgs import ROSmsg


class ObjectType(ROSmsg):
    """https://docs.ros.org/en/api/object_recognition_msgs/html/msg/ObjectType.html"""

    ROS_MSG_TYPE = 'object_recognition_msgs/ObjectType'

    def __init__(self, key='key', db='db'):
        self.key = key
        self.db = db

    @classmethod
    def from_msg(cls, msg):
        return cls(msg['key'], msg['db'])
