import json
from collections.abc import Mapping

from roslibpy import Header as RoslibpyRos1Header
from roslibpy.ros2 import Header as RoslibpyRos2Header

_TYPE_MAP = {}


def _to_plain(value):
    """Recursively convert ``Mapping`` values (e.g. roslibpy's ``UserDict``-based
    ``Header``/``Time``) into plain ``dict``/``list`` so the result is JSON serializable.

    roslibpy 2.0 wraps a header's ``stamp`` in a ``roslibpy.core.Time``, which is a
    ``UserDict`` and *not* a real ``dict`` — ``json.dumps`` rejects it. Flattening here
    keeps the per-distro field shaping roslibpy provides while emitting only plain types.
    """
    if isinstance(value, Mapping):
        return {k: _to_plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_plain(v) for v in value]
    return value


def format_header_for_distro(header, ros_distro):
    if hasattr(header, "filter_fields_for_distro"):
        header.filter_fields_for_distro(ros_distro)
        return header

    stamp = _format_stamp_for_roslibpy(header.get("stamp")) if header else None
    frame_id = header.get("frame_id") if header else None

    if ros_distro.is_ros2:
        return _to_plain(RoslibpyRos2Header(stamp=stamp, frame_id=frame_id))

    seq = header.get("seq") if header else None
    return _to_plain(RoslibpyRos1Header(seq=seq, stamp=stamp, frame_id=frame_id))


def _format_stamp_for_roslibpy(stamp):
    if not stamp:
        return None
    if hasattr(stamp, "msg"):
        stamp = stamp.msg
    if "secs" in stamp and "nsecs" in stamp:
        return stamp
    if "sec" in stamp and "nanosec" in stamp:
        return {"secs": stamp["sec"], "nsecs": stamp["nanosec"]}
    return stamp


class ROSmsg:
    """The base class for ROS messages.

    Attributes
    ----------
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
    """ROS 1: https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Time.html
    ROS 2: https://docs.ros.org/en/jazzy/p/builtin_interfaces/interfaces/msg/Time.html
    """

    ROS_MSG_TYPE = "std_msgs/Time"

    def __init__(self, secs=0, nsecs=0):
        self.secs = secs
        self.nsecs = nsecs

    @classmethod
    def from_msg(cls, msg):
        if "secs" in msg and "nsecs" in msg:
            return cls(msg["secs"], msg["nsecs"])
        return cls(msg["sec"], msg["nanosec"])

    def seconds(self):
        return self.secs + 1e-9 * self.nsecs


class Header(ROSmsg):
    """ROS 1: https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html
    ROS 2: https://docs.ros.org/en/jazzy/p/std_msgs/interfaces/msg/Header.html

    Note: ROS 2's Header drops the `seq` field that ROS 1's has.
    """

    ROS_MSG_TYPE = "std_msgs/Header"

    def __init__(self, seq=0, stamp=Time(), frame_id="/world"):
        self.seq = seq
        self.stamp = stamp
        self.frame_id = frame_id
        self._roslibpy_header_cls = RoslibpyRos1Header

    @property
    def msg(self):
        stamp = _format_stamp_for_roslibpy(self.stamp)
        if self._roslibpy_header_cls is RoslibpyRos2Header:
            return _to_plain(RoslibpyRos2Header(stamp=stamp, frame_id=self.frame_id))
        return _to_plain(RoslibpyRos1Header(seq=self.seq, stamp=stamp, frame_id=self.frame_id))

    def filter_fields_for_distro(self, ros_distro):
        self._roslibpy_header_cls = RoslibpyRos2Header if ros_distro.is_ros2 else RoslibpyRos1Header

    def __repr__(self):
        return "Header(seq={!r}, stamp={!r}, frame_id={!r})".format(self.seq, self.stamp, self.frame_id)


class String(ROSmsg):
    """ROS 1: https://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html
    ROS 2: https://docs.ros.org/en/jazzy/p/std_msgs/interfaces/msg/String.html
    """

    ROS_MSG_TYPE = "std_msgs/String"

    def __init__(self, data=""):
        self.data = data


class MultiArrayDimension(ROSmsg):
    """ROS 1: https://docs.ros.org/en/noetic/api/std_msgs/html/msg/MultiArrayDimension.html
    ROS 2: https://docs.ros.org/en/jazzy/p/std_msgs/interfaces/msg/MultiArrayDimension.html
    """

    ROS_MSG_TYPE = "std_msgs/MultiArrayDimension"

    def __init__(self, label=None, size=0, stride=0):
        self.label = label or ""  # label of given dimension
        self.size = size  # size of given dimension (in type units)
        self.stride = stride  # stride of given dimension

    @classmethod
    def from_msg(cls, msg):
        return cls(msg["label"], msg["size"], msg["stride"])


class MultiArrayLayout(ROSmsg):
    """ROS 1: https://docs.ros.org/en/noetic/api/std_msgs/html/msg/MultiArrayLayout.html
    ROS 2: https://docs.ros.org/en/jazzy/p/std_msgs/interfaces/msg/MultiArrayLayout.html
    """

    ROS_MSG_TYPE = "std_msgs/MultiArrayLayout"

    def __init__(self, dim=None, data_offset=None):
        self.dim = dim or []
        self.data_offset = data_offset or 0

    @classmethod
    def from_msg(cls, msg):
        dim = [MultiArrayDimension.from_msg(d) for d in msg["dim"]]
        return cls(dim, msg["data_offset"])


class Int8MultiArray(ROSmsg):
    """ROS 1: https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Int8MultiArray.html
    ROS 2: https://docs.ros.org/en/jazzy/p/std_msgs/interfaces/msg/Int8MultiArray.html
    """

    ROS_MSG_TYPE = "std_msgs/Int8MultiArray"

    def __init__(self, layout=None, data=None):
        self.layout = layout or MultiArrayLayout()
        self.data = data or []

    @classmethod
    def from_msg(cls, msg):
        layout = MultiArrayLayout.from_msg(msg["layout"])
        return cls(layout, msg["data"])


class Float32MultiArray(ROSmsg):
    """ROS 1: https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32MultiArray.html
    ROS 2: https://docs.ros.org/en/jazzy/p/std_msgs/interfaces/msg/Float32MultiArray.html
    """

    ROS_MSG_TYPE = "std_msgs/Float32MultiArray"

    def __init__(self, layout=None, data=None):
        self.layout = layout or MultiArrayLayout()
        self.data = data or []

    @classmethod
    def from_msg(cls, msg):
        layout = MultiArrayLayout.from_msg(msg["layout"])
        return cls(layout, msg["data"])


class Int32(ROSmsg):
    """ROS 1: https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Int32.html
    ROS 2: https://docs.ros.org/en/jazzy/p/std_msgs/interfaces/msg/Int32.html
    """

    ROS_MSG_TYPE = "std_msgs/Int32"

    def __init__(self, data=None):
        self.data = data or 0

    @classmethod
    def from_msg(cls, msg):
        return cls(msg["data"])
