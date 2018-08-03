from compas.geometry.basic import add_vectors
from compas.geometry.basic import scale_vector

def box_corners(frame, xsize, ysize, zsize):
    """Returns the 8 points that define a box in a plane.

    Args:
        frame (:class:`Frame`): The frame of the box.
        xsize (float): The box's size along the plane's x-axis
        ysize (float): The box's size along the plane's y-axis
        zsize (float): The box's size along the plane's z-axis
    """
    corners = []
    for z in [0, 1]:
        for x, y in zip([0, 1, 1, 0],[0, 0, 1, 1]):
            pt = frame.point[:]
            pt = add_vectors(pt, scale_vector(frame.xaxis, x * xsize))
            pt = add_vectors(pt, scale_vector(frame.yaxis, y * ysize))
            pt = add_vectors(pt, scale_vector(frame.zaxis, z * zsize))
            corners.append(pt)
    return corners

def box_faces():
    """Returns the indices of 8 points that define the faces of a box.
    """
    faces = []
    faces.append([0, 3, 2, 1])
    faces.append([3, 0, 4, 7])
    faces.append([0, 1, 5, 4])
    faces.append([1, 2, 6, 5])
    faces.append([2, 3, 7, 6])
    faces.append([4, 5, 6, 7])
    return faces