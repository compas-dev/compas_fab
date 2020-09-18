from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from .std_msgs import ROSmsg
from .std_msgs import Header

from .geometry_msgs import Pose


class Octomap(ROSmsg):
    """http://docs.ros.org/kinetic/api/octomap_msgs/html/msg/Octomap.html"""

    def __init__(self, header=None, binary=False, id='', resolution=0., data=None):
        self.header = header or Header()  # Header
        self.binary = binary  # Flag to denote a binary (only free/occupied) or full occupancy octree (.bt/.ot file)
        self.id = id  # Class id of the contained octree
        self.resolution = resolution  # Resolution (in m) of the smallest octree nodes
        self.data = data or []  # binary serialization of octree, use conversions.h to read and write octrees


class OctomapWithPose(ROSmsg):
    """http://docs.ros.org/kinetic/api/octomap_msgs/html/msg/OctomapWithPose.html"""

    def __init__(self, header=None, origin=None, octomap=None):
        self.header = header or Header()  # Header
        self.origin = origin or Pose()  # geometry_msgs/Pose  The pose of the octree with respect to the header frame
        self.octomap = octomap or Octomap()  # octomap_msgs/Octomap  The actual octree msg
