from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

import math

from compas.utilities import pairwise

from compas.geometry import Point
from compas.geometry import intersection_plane_plane
from compas.geometry import distance_point_point

from compas.geometry.basic import add_vectors
from compas.geometry.basic import subtract_vectors
from compas.geometry.basic import scale_vector
from compas.geometry.basic import cross_vectors
from compas.geometry.basic import dot_vectors
from compas.geometry.basic import length_vector_xy
from compas.geometry.basic import subtract_vectors_xy

from compas.geometry.queries import is_point_on_segment
from compas.geometry.queries import is_point_on_segment_xy



__all__ = [
    'intersection_plane_circle',
    'intersection_sphere_sphere'
]


def intersection_plane_circle(plane, circle):
    """Computes the intersection of a plane with a circle.

    There are 5 cases of plane-circle intersection : 1) the plane coincides with
    the plane containing the circle, 2) the plane is parallel to the plane
    containing the circle, 3) they do not interect, 4) the plane cuts the circle
    in 2 points (secant) and 5) the plane meets the circle in 1 point (tangent).

    Parameters
    ----------
    plane : tuple
        The base point and normal defining the plane.
    circle : tuple
        center, radius, normal of the circle.

    Returns
    -------
    list
        XYZ coordinates of either 2 (case 4) or 1 intersection point (case 5)
    None
        for cases 1), 2), 3)

    Examples
    --------
    >>>

    References
    --------
    """

    # fr_0, c_1, n_1, r_1
    #fr_0; intersecting frame
    #c_1; center of circle
    #r_1; radii cirlce
    #n_1; normal of circle

    #T_0 = Transformation.from_frame_to_frame(fr_0, Frame([0,0,0], [1,0,0], [0,1,0]))
    #T = Transformation.from_frame_to_frame(Frame([0,0,0], [1,0,0], [0,1,0]), fr_0)

    circle_center, _circle_radius, circle_normal = circle
    circle_plane = circle_center, circle_normal

    p1, p2= intersection_plane_plane(plane, circle_plane)

    circle_p0 = Point(c_1.x, c_1.y, c_1.z)
    line_p1_0 = Point(line_p1[0], line_p1[1], line_p1[2])
    line_p2_0 = Point(line_p2[0], line_p2[1], line_p2[2])

    circle_p0.transform(T_0)
    line_p1_0.transform(T_0)
    line_p2_0.transform(T_0)

    Ax = line_p1_0.x
    Ay = line_p1_0.y

    Bx = line_p2_0.x
    By = line_p2_0.y

    Cx = circle_p0.x
    Cy = circle_p0.y

    R = r_1

    #compute the euclidean distance between A and B
    LAB = math.sqrt( (Bx-Ax)**2+(By-Ay)**2)

    #compute the direction vector D from A to B
    Dx = (Bx-Ax)/LAB
    Dy = (By-Ay)/LAB

    #Now the line equation is x = Dx*t + Ax, y = Dy*t + Ay with 0 <= t <= 1.

    #compute the value t of the closest point to the circle center (Cx, Cy)
    t = Dx*(Cx-Ax) + Dy*(Cy-Ay)

    #This is the projection of C on the line from A to B.

    #compute the coordinates of the point E on line and closest to C
    Ex = t*Dx+Ax
    Ey = t*Dy+Ay

    #compute the euclidean distance from E to C
    LEC = math.sqrt( (Ex-Cx)**2+(Ey-Cy)**2 )

    #test if the line intersects the circle
    if( LEC < R ):
        #compute distance from t to circle intersection point
        dt = math.sqrt( R**2 - LEC**2)

        #compute first intersection point
        Fx = (t-dt)*Dx + Ax
        Fy = (t-dt)*Dy + Ay

        #compute second intersection point
        Gx = (t+dt)*Dx + Ax
        Gy = (t+dt)*Dy + Ay

        p1 = Point(Fx, Fy, 0)
        p2 = Point(Gx, Gy, 0)

        p1.transform(T)
        p2.transform(T)

        return p2, p1

    elif( LEC == R ):
        print("tangent point to circle is E")

        return False, False

    else:
        print("line doesn't touch circle")

        return False, False


def intersection_sphere_sphere(sphere1, sphere2):
    """Computes the intersection of 2 spheres.

    There are 4 cases of sphere-sphere intersection : 1) the spheres intersect
    in a circle, 2) they intersect in a point, 3) they overlap, 4) they do not
    intersect.

    Parameters
    ----------
    sphere1 : tuple
        center, radius of the sphere.
    sphere2 : tuple
        center, radius of the sphere.

    Returns
    -------
    tuple
        case (str): "point", "circle", or "sphere"
        result (tuple):
            point: xyz coordinates
            circle: center, radius, normal
            sphere: center, radius
    None
        for case 4) (no intersection)

    Examples
    --------
    >>> sphere1 = (3.0, 7.0, 4.0), 10.0
    >>> sphere2 = (7.0, 4.0, 0.0), 5.0
    >>> result = intersection_sphere_sphere(sphere1, sphere1)
    >>> if result:
    >>>    case, res = result
    >>>    if case == "circle":
    >>>        center, radius, normal = res
    >>>    elif case == "point":
    >>>        point = res
    >>>    elif case == "sphere":
    >>>        center, radius = res

    References
    --------
    https://gamedev.stackexchange.com/questions/75756/sphere-sphere-intersection-and-circle-sphere-intersection

    """

    center1, radius1 = sphere1
    center2, radius2 = sphere2

    distance = distance_point_point(center1, center2)

    # Case 4: No intersection
    if radius1 + radius2 < distance:
        return None
    # Case 4: No intersection, sphere is within the other sphere
    elif distance + min(radius1, radius2) < max(radius1, radius2):
        return None
    # Case 3: sphere's overlap
    elif radius1 == radius2 and distance == 0:
        return "sphere", sphere1
    # Case 2: point intersection
    elif radius1 + radius2 == distance:
        ipt = subtract_vectors(center2, center1)
        ipt = scale_vector(ipt, radius1/distance)
        ipt = add_vectors(center1, ipt)
        return "point", ipt
    # Case 2: point intersection, smaller sphere is within the bigger
    elif distance + min(radius1, radius2) == max(radius1, radius2):
        if radius1 > radius2:
            ipt = subtract_vectors(center2, center1)
            ipt = scale_vector(ipt, radius1/distance)
            ipt = add_vectors(center1, ipt)
        else:
            ipt = subtract_vectors(center1, center2)
            ipt = scale_vector(ipt, radius2/distance)
            ipt = add_vectors(center2, ipt)
        return "point", ipt
    # Case 1: circle intersection
    else:
        h  = 0.5 + (radius1**2 - radius2**2)/(2 * distance**2)
        ci = subtract_vectors(center2, center1)
        ci = scale_vector(ci, h)
        ci = add_vectors(center1, ci)
        ri = math.sqrt(radius1**2 - h**2 * distance**2)
        normal = scale_vector(subtract_vectors(center2, center1), 1/distance)
        return "circle", (ci, ri, normal)

if __name__ == "__main__":

    plane = (0,0,0), (0,0,1)
    circle = (0,0,0), 5
    circle_normal = (1,0,0)

    print(min(2,4))
    #intersection_plane_circle(plane, circle, circle_normal)

    sphere1 = (3.0, 7.0, 4.0), 10.0
    sphere2 = (7.0, 4.0, 0.0), 5.0
    result = intersection_sphere_sphere(sphere1, sphere1)
    if result:
        case, res = result
        if case == "circle":
            center, radius, normal = res
        elif case == "point":
            point = res
        elif case == "sphere":
            center, radius = res
