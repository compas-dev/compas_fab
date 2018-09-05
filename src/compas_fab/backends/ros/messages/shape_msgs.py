from __future__ import absolute_import

from .std_msgs import ROSmsg

class SolidPrimitive(ROSmsg):
    """http://docs.ros.org/kinetic/api/shape_msgs/html/msg/SolidPrimitive.html
    """
    BOX = 1
    SPHERE = 2
    CYLINDER = 3
    CONE = 4
    BOX_X = 0
    BOX_Y = 1
    BOX_Z = 2
    SPHERE_RADIUS = 0
    CYLINDER_HEIGHT = 0
    CYLINDER_RADIUS = 1
    CONE_HEIGHT = 0
    CONE_RADIUS = 1

    def __init__(self, type=1, dimensions=[1, 1, 1]):
        self.type = type
        self.dimensions = dimensions


class Mesh(ROSmsg):
    """http://docs.ros.org/kinetic/api/shape_msgs/html/msg/Mesh.html
    """

    def __init__(self, triangles=[], vertices=[]):
        self.triangles = triangles
        self.vertices = vertices


class Plane(ROSmsg):
    """http://docs.ros.org/kinetic/api/shape_msgs/html/msg/Plane.html
    """

    def __init__(self, coef):
        self.coef = coef
