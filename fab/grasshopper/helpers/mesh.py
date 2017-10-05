from __future__ import print_function
import sys

try:
    import rhinoscriptsyntax as rs
    import ghpythonlib.components as ghcomp
except ImportError:
    import platform
    if platform.python_implementation() == 'IronPython':
        raise
    
from compas.datastructures.mesh import Mesh
from compas_fabrication.fabrication.grasshopper import xdraw_mesh
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


def draw_mesh(mesh,
              layer=None,
              show_faces=True,
              show_vertices=False,
              show_edges=False,
              show_wireframe=False,
              vertexcolor=None,
              edgecolor=None,
              wireframecolor=None,
              facecolor=None,
              ):
    """
    Draw a mesh object in Grasshopper.

    Parameters:
        mesh (compas.datastructures.mesh.Mesh): The mesh object.
        show_faces (bool): Optional. Show the faces. Default is ``True``.
        show_vertices (bool): Optional. Show the vertices. Default is ``True``.
        show_edges (bool): Optional. Show the edges. Default is ``True``.
        vertexcolor (str, tuple, list, dict): Optional. The vertex color specification. Default is ``None``.
        edgecolor (str, tuple, list, dict): Optional. The edge color specification. Default is ``None``.
        facecolor (str, tuple, list, dict): Optional. The face color specification. Default is ``None``.
        redraw (bool): Optional. Redraw instructions. Default is ``True``.

    Note:
        Colors can be specified in different ways:

        * str: A hexadecimal color that will be applied to all elements subject to the specification.
        * tuple, list: RGB color that will be applied to all elements subject to the specification.
        * dict: RGB or hex color dict with a specification for some or all of the related elements.

    Important:
        RGB colors should specify color values between 0 and 255.

    """

    vertexcolor = color_to_colordict(vertexcolor,
                                     mesh.vertices(),
                                     default=mesh.attributes['color.vertex'],
                                     colorformat='rgb',
                                     normalize=False)

    edgecolor = color_to_colordict(edgecolor,
                                   mesh.edges(),
                                   default=mesh.attributes['color.edge'],
                                   colorformat='rgb',
                                   normalize=False)

    # facecolor = color_to_colordict(facecolor,
    #                                mesh.faces(),
    #                                default=mesh.attributes['color.face'],
    #                                colorformat='rgb',
    #                                normalize=False)

    if show_faces:
        key_index = {key: index for index, key in enumerate(mesh.vertices())}
        xyz       = [mesh.vertex_coordinates(key) for key in mesh.vertices()]
        faces     = []
        color     = mesh.attributes['color.face']

        for fkey in mesh.face:
            face = mesh.face_vertices(fkey, ordered=True)
            v = len(face)

            if v < 3:
                print('Degenerate face: {0} => {1}'.format(fkey, face))
            elif v == 3:
                faces.append([key_index[k] for k in face + [face[-1]]])
            elif v == 4:
                faces.append([key_index[k] for k in face])
            else:
                c = len(xyz)
                xyz.append(mesh.face_center(fkey))
                for i in range(-1, len(face) - 1):
                    key = face[i]
                    nbr = face[i + 1]
                    vertices = [c, key_index[key], key_index[nbr], key_index[nbr]]
                    faces.append(vertices)

        return xdraw_mesh(xyz, faces, color)
    
    
#     if show_edges:
#         lines = []
#         color = mesh.attributes['color.edge']
#         for u, v in mesh.edges():
#             lines.append({
#                 'start': mesh.vertex_coordinates(u),
#                 'end'  : mesh.vertex_coordinates(v),
#                 'name' : '{0}.edge.{1}-{2}'.format(mesh.attributes['name'], repr(u), repr(v)),
#                 'color': edgecolor.get((u, v), color),
#             })
#         xdraw_lines(lines)
# 
#     if show_wireframe:
#         lines = []
#         color = mesh.attributes['color.edge']
#         for u, v in mesh.wireframe():
#             lines.append({
#                 'start': mesh.vertex_coordinates(u),
#                 'end'  : mesh.vertex_coordinates(v),
#                 'name' : '{0}.edge.{1}-{2}'.format(mesh.attributes['name'], repr(u), repr(v)),
#                 'color': edgecolor.get((u, v), color),
#             })
#         xdraw_lines(lines)
# 
#     if show_vertices:
#         points = []
#         color  = mesh.attributes['color.vertex']
#         for key in mesh.vertices():
#             points.append({
#                 'pos'  : mesh.vertex_coordinates(key),
#                 'name' : '{0}.vertex.{1}'.format(mesh.attributes['name'], repr(key)),
#                 'color': vertexcolor.get(key, color),
#             })
#         xdraw_points(points)
