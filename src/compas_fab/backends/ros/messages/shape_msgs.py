from __future__ import absolute_import

from .std_msgs import ROSmsg
from .geometry_msgs import Point

import compas.datastructures
from compas.datastructures import mesh_quads_to_triangles


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

    def __init__(self, type=None, dimensions=None):
        self.type = type if type else self.BOX
        self.dimensions = dimensions if dimensions else [1, 1, 1]
        if self.type == self.BOX and len(self.dimensions) != 3:
            raise ValueError("BOX needs 3 dimensions")
        elif self.type == self.SPHERE and len(dimensions) != 1:
            raise ValueError("SPHERE needs 1 dimension.")
        elif self.type == self.CYLINDER and len(dimensions) != 2:
            raise ValueError("CYLINDER needs 2 dimensions.")
        elif self.type == self.CONE and len(dimensions) != 2:
            raise ValueError("CONE needs 2 dimensions.")


class Mesh(ROSmsg):
    """http://docs.ros.org/kinetic/api/shape_msgs/html/msg/Mesh.html
    """
    def __init__(self, triangles=None, vertices=None):
        self.triangles = triangles if triangles else [] # shape_msgs/MeshTriangle[]
        self.vertices = vertices if vertices else [] # geometry_msgs/Point[]

    @classmethod
    def from_mesh(cls, compas_mesh):
        """Construct a ROS Mesh message from a COMPAS Mesh.
        """
        mesh_quads_to_triangles(compas_mesh)
        vertices, faces = compas_mesh.to_vertices_and_faces()
        triangles = [MeshTriangle(face) for face in faces]
        vertices = [Point(*v) for v in vertices]
        return cls(triangles, vertices)

    @classmethod
    def from_msg(cls, msg):
        triangles = [MeshTriangle.from_msg(t) for t in msg['triangles']]
        vertices = [Point.from_msg(v) for v in msg['vertices']]
        return cls(triangles, vertices)

    @property
    def mesh(self):
        cls = compas.datastructures.Mesh
        vertices = [(v.x, v.y, v.z) for v in self.vertices]
        faces = [t.vertex_indices for t in self.triangles]
        return cls.from_vertices_and_faces(vertices, faces)

class MeshTriangle(ROSmsg):
    """http://docs.ros.org/api/shape_msgs/html/msg/MeshTriangle.html
    """
    def __init__(self, vertex_indices=None):
        if len(vertex_indices) != 3:
            raise ValueError("Please specify 3 indices for face, %d given." % len(vertex_indices))
        self.vertex_indices = vertex_indices if vertex_indices else [] # uint32[3]

    @classmethod
    def from_msg(cls, msg):
        vertex_indices = msg['vertex_indices']
        return cls(vertex_indices)

class Plane(ROSmsg):
    """http://docs.ros.org/kinetic/api/shape_msgs/html/msg/Plane.html
    """

    def __init__(self, coef):
        self.coef = coef

