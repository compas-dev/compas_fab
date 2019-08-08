from __future__ import absolute_import

from .std_msgs import ROSmsg


class ObjectType(ROSmsg):
    """http://docs.ros.org/kinetic/api/object_recognition_msgs/html/msg/ObjectType.html
    """

    def __init__(self, key='key', db='db'):
        self.key = key
        self.db = db

    @classmethod
    def from_msg(cls, msg):
        return cls(msg['key'], msg['db'])
