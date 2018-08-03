from __future__ import print_function
import sys

try:
    import rhinoscriptsyntax as rs
    import ghpythonlib.components as ghcomp
    from Rhino.Geometry import Transform
except ImportError:
    import platform
    if platform.python_implementation() == 'IronPython':
        raise

from compas.datastructures.mesh import Mesh


# TODO: This file should actually move to compas_rhino

def xform_from_transformation(transformation):
    """Creates a Rhino Transform instance from a transformation object.

    Args:
        transformation (:class:`Transformation`): the transformation.

    Returns:
        Transform: a Rhino class.
    """
    transform = Transform(1.0)
    for i in range(0, 4):
        for j in range(0, 4):
            transform[i, j] = transformation[i, j]
    return transform

def xtransform(geo, transformation, copy=False):
    """Transforms Rhino Geometry object with a transformation object.

    Args:
        geo (:class:`Rhino.Geometry`): a geometry of Rhino Geometry
        transformation (:class:`Transformation`): the transformation.

    Returns:
        tgeo (:class:`Rhino.Geometry`): the transformed geometry
    """
    T = xform_from_transformation(transformation)
    if copy:
        return ghcomp.Transform(geo, T)
    else:
        geo.Transform(T)
        return geo

def xform_from_matrix(matrix):
    """Creates a Transform instance from a matrix represented as a list
    of 12 float values.
    """
    transform = Transform(1.0)
    for i in range(0, len(matrix)):
        transform[i // 4, i % 4] = matrix[i]

    return transform

# TODO: vrep is very specific, better rename as the return is anyway not a proprietary type
def vrep_pose_from_plane(plane):
    """Creates a vrep-compatible transformation matrix from a Rhino/Grasshopper
    plane.

    This function might need rework as the source of the 90-deg Y rotation
    need is not entirely clear to me (related to the RFL model mismatch).
    """
    translation_matrix = rs.XformTranslation(((plane[0][0]), (plane[0][1]), plane[0][2]))
    plane_start = rs.PlaneFromFrame(rs.AddPoint(0, 0, 0), rs.AddPoint(1, 0, 0), rs.AddPoint(0, 1, 0))
    plane_end = rs.PlaneFromFrame(rs.AddPoint(0, 0, 0), rs.AddPoint(plane[1][0], (plane[1][1]), plane[1][2]), rs.AddPoint(plane[2][0], plane[2][1], plane[2][2]))
    rotation_matrix = rs.XformRotation1(plane_start, plane_end)
    matrix = rs.XformMultiply(translation_matrix, rotation_matrix)
    return [matrix.M00, matrix.M01, matrix.M02, matrix.M03,
            matrix.M10, matrix.M11, matrix.M12, matrix.M13,
            matrix.M20, matrix.M21, matrix.M22, matrix.M23]
