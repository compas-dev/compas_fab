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
            if hasattr(value, "msg"):
                msg[key] = value.msg
            elif isinstance(value, list):
                if len(value):
                    if hasattr(value[0], "msg"):
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
            args.append("{}={!r}".format(key, value))

        return "{}({})".format(self.__class__.__name__, ", ".join(args))

    @staticmethod
    def parse(msg, msg_type):
        """Parses a msg in various possible input formats and tries to return an instance of the class that wraps it.

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
            raise Exception("Unsupported message type, use either a ROSmsg, a dict, or a json string")

        wrapper_cls = _TYPE_MAP.get(msg_type, ROSmsg)

        return wrapper_cls.from_msg(msg)


class Time(ROSmsg):
    """https://docs.ros.org/kinetic/api/std_msgs/html/msg/Time.html"""

    ROS_MSG_TYPE = "std_msgs/Time"

    def __init__(self, secs=0, nsecs=0):
        self.secs = secs
        self.nsecs = nsecs

    def seconds(self):
        return self.secs + 1e-9 * self.nsecs


class Header(ROSmsg):
    """https://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html"""

    ROS_MSG_TYPE = "std_msgs/Header"

    def __init__(self, seq=0, stamp=Time(), frame_id="/world"):
        self.seq = seq
        self.stamp = stamp
        self.frame_id = frame_id


class String(ROSmsg):
    """https://docs.ros.org/api/std_msgs/html/msg/String.html"""

    ROS_MSG_TYPE = "std_msgs/String"

    def __init__(self, data=""):
        self.data = data


class MultiArrayDimension(ROSmsg):
    """http://docs.ros.org/en/api/std_msgs/html/msg/MultiArrayDimension.html"""

    ROS_MSG_TYPE = "std_msgs/MultiArrayDimension"

    def __init__(self, label=None, size=0, stride=0):
        self.label = label or ""  # label of given dimension
        self.size = size  # size of given dimension (in type units)
        self.stride = stride  # stride of given dimension

    @classmethod
    def from_msg(cls, msg):
        return cls(msg["label"], msg["size"], msg["stride"])


class MultiArrayLayout(ROSmsg):
    """http://docs.ros.org/en/api/std_msgs/html/msg/MultiArrayLayout.html"""

    ROS_MSG_TYPE = "std_msgs/MultiArrayLayout"

    def __init__(self, dim=None, data_offset=None):
        self.dim = dim or []
        self.data_offset = data_offset or 0

    @classmethod
    def from_msg(cls, msg):
        dim = [MultiArrayDimension.from_msg(d) for d in msg["dim"]]
        return cls(dim, msg["data_offset"])


class Int8MultiArray(ROSmsg):
    """http://docs.ros.org/en/api/std_msgs/html/msg/Int8MultiArray.html"""

    ROS_MSG_TYPE = "std_msgs/Int8MultiArray"

    def __init__(self, layout=None, data=None):
        self.layout = layout or MultiArrayLayout()
        self.data = data or []

    @classmethod
    def from_msg(cls, msg):
        layout = MultiArrayLayout.from_msg(msg["layout"])
        return cls(layout, msg["data"])


class Float32MultiArray(ROSmsg):
    """http://docs.ros.org/en/api/std_msgs/html/msg/Float32MultiArray.html"""

    ROS_MSG_TYPE = "std_msgs/Float32MultiArray"

    def __init__(self, layout=None, data=None):
        self.layout = layout or MultiArrayLayout()
        self.data = data or []

    @classmethod
    def from_msg(cls, msg):
        layout = MultiArrayLayout.from_msg(msg["layout"])
        return cls(layout, msg["data"])


class Int32(ROSmsg):
    """http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Int32.html"""

    ROS_MSG_TYPE = "std_msgs/Int32"

    def __init__(self, data=None):
        self.data = data or 0

    @classmethod
    def from_msg(cls, msg):
        return cls(msg["data"])
