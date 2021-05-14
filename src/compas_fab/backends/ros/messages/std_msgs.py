from __future__ import absolute_import
from __future__ import print_function

import json

import compas

if compas.PY3:
    from collections.abc import Mapping
else:
    from collections import Mapping

_TYPE_MAP = {}


class ROSmsg(object):
    """The base class for ROS messages.

    Class Attributes
    ----------------
    ROS_MSG_TYPE : :obj:`str`
        Name of the message type that the class represents. Sub-classes of ROSmsg must define this type. e.g. ``std_msgs/String``
    """

    ROS_MSG_TYPE = None

    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)

    @property
    def msg(self):
        msg = {}
        for key, value in self.__dict__.items():
            if hasattr(value, 'msg'):
                msg[key] = value.msg
            elif isinstance(value, list):
                if len(value):
                    if hasattr(value[0], 'msg'):
                        msg[key] = [v.msg for v in value]
                    else:
                        msg[key] = value
                else:
                    msg[key] = value
            else:
                msg[key] = value
        return msg

    @classmethod
    def from_msg(cls, msg):
        return cls(**msg)

    def __str__(self):
        return str(self.msg)

    def __repr__(self):
        args = []
        for key, value in self.__dict__.items():
            args.append('{}={!r}'.format(key, value))

        return '{}({})'.format(self.__class__.__name__, ', '.join(args))

    @staticmethod
    def parse(msg, msg_type):
        """Parses a msg in various possible input formats and tries to return an instance of the that wraps it.

        Parameters
        ----------
        msg : :obj:`str` or :obj:`dict`
            Message encoded as a JSON string or as a Python dictionary.
        msg_type: :obj:`str`
            ROS message type, eg. ``std_msgs/String``

        Returns
        -------
        :class:`ROSmsg`
            Instance of the class wrapping the message. Either :class:`ROSmsg` or one of its sub-classes.
        """
        # Build type map if it's the first call
        if not len(_TYPE_MAP):
            for cls in ROSmsg.__subclasses__():
                _TYPE_MAP[cls.ROS_MSG_TYPE] = cls

        if isinstance(msg, str):
            msg = json.loads(msg)

        if not isinstance(msg, Mapping):
            raise Exception('Unsupported message type, use either a ROSmsg, a dict, or a json string')

        wrapper_cls = _TYPE_MAP.get(msg_type, ROSmsg)

        return wrapper_cls.from_msg(msg)


class Time(ROSmsg):
    """https://docs.ros.org/kinetic/api/std_msgs/html/msg/Time.html
    """
    ROS_MSG_TYPE = 'std_msgs/Time'

    def __init__(self, secs=0, nsecs=0):
        self.secs = secs
        self.nsecs = nsecs

    def seconds(self):
        return self.secs + 1e-9 * self.nsecs


class Header(ROSmsg):
    """https://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
    """
    ROS_MSG_TYPE = 'std_msgs/Header'

    def __init__(self, seq=0, stamp=Time(), frame_id='/world'):
        self.seq = seq
        self.stamp = stamp
        self.frame_id = frame_id


class String(ROSmsg):
    """https://docs.ros.org/api/std_msgs/html/msg/String.html
    """
    ROS_MSG_TYPE = 'std_msgs/String'

    def __init__(self, data=''):
        self.data = data
