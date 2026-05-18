from .geometry_msgs import Pose
from .std_msgs import Header
from .std_msgs import ROSmsg
from .std_msgs import format_header_for_distro


class Octomap(ROSmsg):
    """https://docs.ros.org/kinetic/api/octomap_msgs/html/msg/Octomap.html"""

    ROS_MSG_TYPE = "octomap_msgs/Octomap"

    def __init__(self, header=None, binary=False, id="", resolution=0.0, data=None):
        self.header = header or Header()  # Header
        self.binary = binary  # Flag to denote a binary (only free/occupied) or full occupancy octree (.bt/.ot file)
        self.id = id  # Class id of the contained octree
        self.resolution = resolution  # Resolution (in m) of the smallest octree nodes
        self.data = data or []  # binary serialization of octree, use conversions.h to read and write octrees

    def filter_fields_for_distro(self, ros_distro):
        self.header = format_header_for_distro(self.header, ros_distro)


class OctomapWithPose(ROSmsg):
    """https://docs.ros.org/kinetic/api/octomap_msgs/html/msg/OctomapWithPose.html"""

    ROS_MSG_TYPE = "octomap_msgs/OctomapWithPose"

    def __init__(self, header=None, origin=None, octomap=None):
        self.header = header or Header()  # Header
        self.origin = origin or Pose()  # geometry_msgs/Pose  The pose of the octree with respect to the header frame
        self.octomap = octomap or Octomap()  # octomap_msgs/Octomap  The actual octree msg

    def filter_fields_for_distro(self, ros_distro):
        self.header = format_header_for_distro(self.header, ros_distro)
        if hasattr(self.octomap, "filter_fields_for_distro"):
            self.octomap.filter_fields_for_distro(ros_distro)
