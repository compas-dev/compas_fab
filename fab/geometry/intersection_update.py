from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from math import fabs

from compas.utilities import pairwise

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



__author__ = ['Matthias Helmreich', ]
__copyright__ = 'Copyright 2018 - Gramazio Kohler Research, ETH Zurich'
__license__ = 'MIT License'
__email__ = 'helmreich@arch.ethz.ch'


__all__ = [
    'intersection_plane_circle',
    'intersection_sphere_sphere'
]


def intersection_plane_circle(plane, circle, circle_normal):
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
        center, radius of the circle in the xy plane.
    circle_normal : tuple
        the normal of the plane containing the circle

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

    p1 = Plane.from_point_and_normal(c_1, n_1.normal)
    p2 = Plane.from_point_and_two_vectors(Point(fr_0.point[0], fr_0.point[1], fr_0.point[2]), Vector(fr_0.xaxis[0], fr_0.xaxis[1], fr_0.xaxis[2]),  Vector(fr_0.yaxis[0], fr_0.yaxis[1], fr_0.yaxis[2]))
    
    line_p1, line_p2= intersection_plane_plane(p1, p2)

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
    LAB = m.sqrt( (Bx-Ax)**2+(By-Ay)**2)
    
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
    LEC = m.sqrt( (Ex-Cx)**2+(Ey-Cy)**2 )
    
    #test if the line intersects the circle
    if( LEC < R ):
        #compute distance from t to circle intersection point
        dt = m.sqrt( R**2 - LEC**2)
    
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
        case (str): "circle" or "point"
        result (tuple): 
            circle: center, radius, normal
            point: xyz coordinates
    None
        for case 3) (overlap) and case 4) (no intersection)

    Examples
    --------
    >>>

    References
    --------
    https://gamedev.stackexchange.com/questions/75756/sphere-sphere-intersection-and-circle-sphere-intersection

    d; abs distance between c_1, c_2
    """

    #c_1; center of first circle    
    #r_1; radii of first cirlce
    #c_2; center of 2nd circle
    #r_2; radii of 2nd cirlce

    center1, radius1 = sphere1
    center2, radius2 = sphere2

    d = distance_point_point(center1, center2)

    if r_1 + r_2 < d:
        print("d=%s; no intersection between the spheres" % round(d, 2))
        return None, None, None
    elif d + min(r_1, r_2) < max(r_1, r_2):
        print("no intersection, circle lies within a circle")
        return None, None, None
    elif r_1 + r_2 == d:
        c_i = c_1 + (c_2 - c_1) * r_1/d
        print("intersection is a single point")
        return c_i, None, None
    elif d + min(r_1, r_2) == max(r_1, r_2):
        c_i = c_larger + (c_smaller - c_larger) * r_larger/d
        print("single point of intersection lies on the larger sphere")
        return c_i, None, None
    else:
        h = 0.5 + (r_1 * r_1 - r_2 * r_2)/(2 * d*d)

        c_1 = Point(c_1[0], c_1[1], c_1[2])
        c_2 = Point(c_2[0], c_2[1], c_2[2])
        c_i = c_1 + (c_2 - c_1).__imul__(h)
        r_i = m.sqrt(r_1*r_1 - h*h*d*d)
        #n_i = (c_2 - c_1).__imul__(1/d)
        n_normal = (c_2 - c_1)
        n_i = Plane.from_point_and_normal(c_i, n_normal)
        
        
        #following describes a point within the circle on the plane n_i
        """
        p_i(theta) = c_i + r_i * (t_i * cos(theta) + b_i sin(theta))
        t_i = normalize(cross(axis, n_i))
        b_i = cross(t_i, n_i)
        """

        return c_i, r_i, n_i

if __name__ == "__main__":

    plane = (0,0,0), (0,0,1)
    circle = (0,0,0), 5
    circle_normal = (1,0,0)

    intersection_plane_circle(plane, circle, circle_normal)
