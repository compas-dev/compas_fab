'''
Created on 19.06.2017

@author: rustr
'''

"""
These methods are temporary placed here until they are implemented in compas library.
"""

import Rhino
import System
import rhinoscriptsyntax as rs

from compas.utilities.colors import color_to_colordict


def mesh_transform(mesh, transformation, copy=True):
    """ 
    Should this be part of Mesh class?
    """
    if copy:
        mesh = mesh.copy()
    for key, attr in mesh.vertices_iter(True):
        p = [attr['x'], attr['y'], attr['z']]
        p = transformation * p
        mesh.vertex[key].update({'x':p[0], 'y':p[1], 'z':p[2]})
    return mesh



def xdraw_mesh(vertices, faces, vertex_normals=None, texture_coordinates=None, vertex_colors=None):
    """ 
    This should move to compas/compas_grasshoper/utilities/drawing.py. 
    
    This function is mainly a copy of rhinoscriptsyntax.AddMesh, just without scriptcontext adding.
    
    """
    
    mesh = Rhino.Geometry.Mesh()
    for a, b, c in vertices: 
        mesh.Vertices.Add(a, b, c)
    for face in faces:
        if len(face)<4:
            mesh.Faces.AddFace(face[0], face[1], face[2])
        else:
            mesh.Faces.AddFace(face[0], face[1], face[2], face[3])
    
    if vertex_normals:
        count = len(vertex_normals)
        normals = System.Array.CreateInstance(Rhino.Geometry.Vector3f, count)
        for i, normal in enumerate(vertex_normals):
            normals[i] = Rhino.Geometry.Vector3f(normal[0], normal[1], normal[2])
        mesh.Normals.SetNormals(normals)
        
    if texture_coordinates:
        count = len(texture_coordinates)
        tcs = System.Array.CreateInstance(Rhino.Geometry.Point2f, count)
        for i, tc in enumerate(texture_coordinates):
            tcs[i] = Rhino.Geometry.Point2f(tc[0], tc[1])
        mesh.TextureCoordinates.SetTextureCoordinates(tcs)
    
    if vertex_colors:
        for i, color in vertex_colors.iteritems():
            color = rs.coercecolor(color)
            mesh.VertexColors.SetColor(i, color)
            
    return mesh


def draw_line(line):
    start = Rhino.Geometry.Point3d(*line.start)
    end = Rhino.Geometry.Point3d(*line.end)
    return Rhino.Geometry.Line(start, end)

def draw_frame(frame):
    pt  = Rhino.Geometry.Point3d(*frame.point)
    xaxis = Rhino.Geometry.Point3d(*frame.xaxis)
    yaxis = Rhino.Geometry.Point3d(*frame.yaxis)
    return Rhino.Geometry.Plane(pt, xaxis, yaxis)



def draw_mesh(mesh, vertexcolor=None):
    """
    This file should be in compas/compas_grasshopper/helpers/mesh.py (like compas/compas_rhino/helpers/mesh.py)
    
    Draw a mesh object in GH.

    Parameters:
        mesh (compas.datastructures.mesh.Mesh): The mesh object.
        vertexcolor (str, tuple, list, dict): Optional. The vertex color specification. Default is ``None``.
        
    Note:
        Colors can be specifiedin different ways:

        * str: A hexadecimal color that will be applied to all elements subject to the specification.
        * tuple, list: RGB color that will be applied to all elements subject to the specification.
        * dict: RGB or hex color dict with a specification for some or all of the related elements.

    Important:
        RGB colors should specify color values between 0 and 255.
        
    """
    
    # set default options
    vertexcolor = color_to_colordict(vertexcolor,
                                     mesh.vertices(),
                                     default=mesh.attributes['color.vertex'],
                                     colorformat='rgb',
                                     normalize=False)
        
    key_index = dict((key, index) for index, key in mesh.vertices_enum())
    xyz       = [mesh.vertex_coordinates(key) for key in mesh.vertices_iter()]
    faces     = []
        
    for fkey in mesh.face:
        face = mesh.face_vertices(fkey, ordered=True)
        v = len(face)
        if v < 3:
            print('Degenerate face: {0} => {1}'.format(fkey, face))
            continue
        if v == 3:
            faces.append([key_index[k] for k in face + [face[-1]]])
        elif v == 4:
            faces.append([key_index[k] for k in face])
        else:
            # a polygonal face
            # => triangulate
            c = len(xyz)
            xyz.append(mesh.face_center(fkey))
            for i in range(-1, len(face) - 1):
                key = face[i]
                nbr = face[i + 1]
                vertices = [c, key_index[key], key_index[nbr], key_index[nbr]]
                faces.append(vertices)
        
    return xdraw_mesh(xyz, faces, vertex_colors=vertexcolor)

    

        