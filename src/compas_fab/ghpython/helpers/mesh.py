from __future__ import print_function

try:
    import rhinoscriptsyntax as rs
    import ghpythonlib.components as ghcomp
except ImportError:
    import platform
    if platform.python_implementation() == 'IronPython':
        raise

from compas.datastructures.mesh import Mesh
from compas.utilities.colors import color_to_colordict


def mesh_from_guid(guid, **kwargs):
    """Creates an instance of a compAS mesh class from an identifier
    in Rhino/Grasshopper.

    This function is almost identical to ``mesh_from_guid`` in the core
    framework, but there were some import issues when used from within
    Grasshopper, but eventually, it should be migrated into the core.
    """
    trimesh = ghcomp.Triangulate(rs.coercemesh(guid))[0]
    vertices = [map(float, vertex) for vertex in rs.MeshVertices(trimesh)]
    faces = map(list, rs.MeshFaceVertices(trimesh))
    faces = [face[: -1] if face[-2] == face[-1] else face for face in faces]
    mesh = Mesh.from_vertices_and_faces(vertices, faces)
    mesh.attributes.update(kwargs)
    return mesh
