import math

from compas_fab.fab.geometry import Frame
from compas.geometry import length_vector
from compas.geometry import subtract_vectors
from compas.geometry import add_vectors
from compas.geometry import scale_vector
from compas.geometry import angle_vectors
from compas.geometry.transformations import matrix_from_axis_and_angle
from compas.geometry.transformations import transform


def inverse_kinematics_four_point_method(P1, P2, P3, P4, target_frame, base_point = [0, 0, 0]):
    """Four point inverse kinematic calculation from
    http://www.grasshopper3d.com/group/lobster/forum/topics/lobster-reloaded
    for certain robot models. Points P1-P4 must be defined in a certain 
    configuration, please refer to http://www.grasshopper3d.com/group/lobster
    P2, P3, P4 must be in XZ plane.
    """
    # TODO: change to compas!

    import Rhino.Geometry as rg
    import rhinoscriptsyntax as rs
    
    P1x = rg.Point3d(P1[0], 0, P1[2]) # P1 in XZ plane
    P1 = rg.Point3d(*P1)
    P2 = rg.Point3d(*P2)
    P3 = rg.Point3d(*P3)
    P4 = rg.Point3d(*P4)
    
    end_frame = rg.Plane(rg.Point3d(*target_frame.point), rg.Vector3d(*target_frame.yaxis), rg.Vector3d(*target_frame.xaxis))

    wrist_offset = P4[0] - P3[0]
    lower_arm_length = P2[2] - P1[2]
    upper_arm_length = P2.DistanceTo(P3)
    axis4_offset_angle = math.atan2(P3[2] - P2[2], P3[0] - P2[0])

    wrist = rg.Point3d(end_frame.PointAt(0, 0, wrist_offset))

    A1 = []
    A2 = []
    A3 = []
    A4 = []
    A5 = []
    A6 = []
    
    v1 = rg.Point3d(wrist.X, wrist.Y, 0) - rg.Point3d(base_point[0], base_point[1], 0)
    v2 = rg.Point3d(P1x.X, P1x.Y, 0) - rg.Point3d(base_point[0], base_point[1], 0)
    
    
    if v2.Length:
        if v1.Length < v2.Length:
            alpha = 0
            raise NotImplementedError # todo: what if wrist pt inside circle
        alpha = math.acos(v2.Length/v1.Length)
        beta = math.radians(rs.VectorAngle(v2, v1))
        if math.fabs(math.atan2(wrist.Y, wrist.X)) > math.pi/2:
            A1 = [beta - alpha] * 4 + [beta + alpha] * 4
        else:
            A1 = [-1 * (beta - alpha)] * 4 + [-1 * (beta + alpha)] * 4
    else:
        a1_angle = -1 * math.atan2(wrist.Y, wrist.X)
        
        if(a1_angle > math.pi):
          a1_angle -= 2 * math.pi
        
        A1 += [a1_angle] * 4
        
        a1_angle += math.pi
        if(a1_angle > math.pi):
          a1_angle -= 2 * math.pi
        
        A1 += [a1_angle] * 4
    

    for i in range(2):
        a1_angle = A1[i * 4]
        
        Rot1 = rg.Transform.Rotation(-1 * a1_angle, rg.Point3d(*base_point))
                
        P1A = rg.Point3d(P1)
        P1xA = rg.Point3d(P1x)
        P2A = rg.Point3d(P2)
        P3A = rg.Point3d(P3)
        
        P1A.Transform(Rot1)
        P1xA.Transform(Rot1)
        P2A.Transform(Rot1)
        P3A.Transform(Rot1)
                
        elbow_dir = rg.Vector3d(1, 0, 0)
        elbow_dir.Transform(Rot1)
        
        elbow_frame = rg.Plane(P1xA, elbow_dir, rg.Vector3d.ZAxis)
        
        sphere1 = rg.Sphere(P1A, lower_arm_length)
        sphere2 = rg.Sphere(wrist, upper_arm_length)

        ok, circle = rg.Intersect.Intersection.SphereSphere(sphere1, sphere2)
        ok, t1, t2 = rg.Intersect.Intersection.PlaneCircle(elbow_frame, circle)

        intersection_pt1 = circle.PointAt(t1)
        intersection_pt2 = circle.PointAt(t2)

        for j in range(2):
            if(j == 0):
                elbow_pt = intersection_pt1
            else:
                elbow_pt = intersection_pt2

            ok, elbowx, elbowy = elbow_frame.ClosestParameter(elbow_pt)
            ok, wristx, wristy = elbow_frame.ClosestParameter(wrist)
            
            a2_angle = math.atan2(elbowy, elbowx)
            a3_angle = math.pi - a2_angle + math.atan2(wristy - elbowy, wristx - elbowx) - axis4_offset_angle
            
            for k in range(2):
                A2.append(-a2_angle)
                a3_angle_wrapped = -a3_angle + math.pi
                while (a3_angle_wrapped >= math.pi):
                    a3_angle_wrapped -= 2 * math.pi
                while (a3_angle_wrapped < -math.pi):
                    a3_angle_wrapped += 2 * math.pi
                A3.append(a3_angle_wrapped)

            for k in range(2):

                axis4 = rg.Vector3d(wrist - elbow_pt)
                axis4.Rotate(-axis4_offset_angle, elbow_frame.ZAxis)
                lower_arm = rg.Vector3d(elbow_pt - P1xA)
                temp_frame = rg.Plane(elbow_frame)
                temp_frame.Rotate(a2_angle + a3_angle, temp_frame.ZAxis)
                axis4_frame = rg.Plane(wrist, temp_frame.ZAxis, -1.0 * temp_frame.YAxis)
                
                ok, axis6x, axis6y = axis4_frame.ClosestParameter(end_frame.Origin)
                
                a4_angle = math.atan2(axis6y, axis6x)
                if k == 1:
                    a4_angle += math.pi
                    if(a4_angle > math.pi):
                        a4_angle -= 2 * math.pi
                
                a4_angle_wrapped = a4_angle + math.pi / 2
                while (a4_angle_wrapped >= math.pi):
                    a4_angle_wrapped -= 2 * math.pi
                while (a4_angle_wrapped < -math.pi):
                    a4_angle_wrapped += 2 * math.pi
                
                A4.append(a4_angle_wrapped)
                axis5_frame = rg.Plane(axis4_frame)
                axis5_frame.Rotate(a4_angle, axis4_frame.ZAxis)
                axis5_frame = rg.Plane(wrist, -axis5_frame.ZAxis, axis5_frame.XAxis)
                ok, axis6x, axis6y = axis5_frame.ClosestParameter(end_frame.Origin)
                a5_angle = math.atan2(axis6y, axis6x)
                A5.append(a5_angle)
                
                axis6_frame = rg.Plane(axis5_frame)
                axis6_frame.Rotate(a5_angle, axis5_frame.ZAxis)
                axis6_frame = rg.Plane(wrist, -axis6_frame.YAxis, axis6_frame.ZAxis)
                ok, endx, endy = axis6_frame.ClosestParameter(end_frame.PointAt(1, 0))
                a6_angle = math.atan2(endy, endx)
                A6.append(a6_angle)

    for i in range(8):
        A1[i] = (A1[i] * -1) + math.pi
        A2[i] = (A2[i] + math.pi/2) * -1
        A3[i] = A3[i] * -1 #(A3[i] - math.pi/2) * -1 - math.pi + math.pi/2
        A4[i] = A4[i] * -1
        A5[i] = A5[i]
        A6[i] = A6[i] * -1

    """
    q0 += math.pi
    q1 *= -1
    q2 *= -1
    q2 -= math.pi/2
    """

    return A1, A2, A3, A4, A5, A6


def inverse_kinematics_four_point_method_compas(P1, P2, P3, P4, target_frame, base_point = [0, 0, 0]):
    """Four point inverse kinematic calculation from
    http://www.grasshopper3d.com/group/lobster/forum/topics/lobster-reloaded
    for certain robot models. Points P1-P4 must be defined in a certain 
    configuration, please refer to http://www.grasshopper3d.com/group/lobster
    P2, P3, P4 must be in XZ plane.
    """
    
    P1x = [P1[0], 0, P1[2]] # P1 in XZ plane
    
    end_frame = Frame(target_frame.point, target_frame.yaxis, target_frame.xaxis)

    wrist_offset = P4[0] - P3[0]
    lower_arm_length = P2[2] - P1[2]
    upper_arm_length = length_vector(subtract_vectors(P2, P3))
    axis4_offset_angle = math.atan2(P3[2] - P2[2], P3[0] - P2[0])

    wrist = add_vectors(end_frame.point, scale_vector(end_frame.zaxis, wrist_offset))

    A1 = []
    A2 = []
    A3 = []
    A4 = []
    A5 = []
    A6 = []
    
    v1 = subtract_vectors([wrist[0], wrist[1], 0], base_point)
    v2 = subtract_vectors([P1x[0], P1x[1], 0], base_point)
    
    v1_length = length_vector(v1)
    v2_length = length_vector(v2)
    
    if v2_length:
        if v1_length < v2_length:
            alpha = 0
            raise NotImplementedError # todo: what if wrist pt inside circle
        alpha = math.acos(v2_length/v1_length)
        beta = angle_vectors(v2, v1)
        if math.fabs(math.atan2(wrist.Y, wrist.X)) > math.pi/2:
            A1 = [beta - alpha] * 4 + [beta + alpha] * 4
        else:
            A1 = [-1 * (beta - alpha)] * 4 + [-1 * (beta + alpha)] * 4
    else:
        a1_angle = -1 * math.atan2(wrist.Y, wrist.X)
        
        if(a1_angle > math.pi):
          a1_angle -= 2 * math.pi
        
        A1 += [a1_angle] * 4
        
        a1_angle += math.pi
        if(a1_angle > math.pi):
          a1_angle -= 2 * math.pi
        
        A1 += [a1_angle] * 4
    

    for i in range(2):
        a1_angle = A1[i * 4]
        
        R1 = matrix_from_axis_and_angle([0, 0, 1], -1 * a1, base_point)

        P1A, P1xA, P2A, P3A = transform([P1, P1x, P2, P3], R1)

        elbow_dir = transform([[1, 0, 0]], R1)[0]
        elbow_frame = Frame(P1xA, elbow_dir, [0, 0, 1])

        # sphere, sphere intersection compas not jet implemented
        
        sphere1 = rg.Sphere(P1A, lower_arm_length)
        sphere2 = rg.Sphere(wrist, upper_arm_length)

        ok, circle = rg.Intersect.Intersection.SphereSphere(sphere1, sphere2)
        ok, t1, t2 = rg.Intersect.Intersection.PlaneCircle(elbow_frame, circle)

        intersection_pt1 = circle.PointAt(t1)
        intersection_pt2 = circle.PointAt(t2)

        for j in range(2):
            if(j == 0):
                elbow_pt = intersection_pt1
            else:
                elbow_pt = intersection_pt2

            ok, elbowx, elbowy = elbow_frame.ClosestParameter(elbow_pt)
            ok, wristx, wristy = elbow_frame.ClosestParameter(wrist)
            
            a2_angle = math.atan2(elbowy, elbowx)
            a3_angle = math.pi - a2_angle + math.atan2(wristy - elbowy, wristx - elbowx) - axis4_offset_angle
            
            for k in range(2):
                A2.append(-a2_angle)
                a3_angle_wrapped = -a3_angle + math.pi
                while (a3_angle_wrapped >= math.pi):
                    a3_angle_wrapped -= 2 * math.pi
                while (a3_angle_wrapped < -math.pi):
                    a3_angle_wrapped += 2 * math.pi
                A3.append(a3_angle_wrapped)

            for k in range(2):

                axis4 = rg.Vector3d(wrist - elbow_pt)
                axis4.Rotate(-axis4_offset_angle, elbow_frame.ZAxis)
                lower_arm = rg.Vector3d(elbow_pt - P1xA)
                temp_frame = rg.Plane(elbow_frame)
                temp_frame.Rotate(a2_angle + a3_angle, temp_frame.ZAxis)
                axis4_frame = rg.Plane(wrist, temp_frame.ZAxis, -1.0 * temp_frame.YAxis)
                
                ok, axis6x, axis6y = axis4_frame.ClosestParameter(end_frame.Origin)
                
                a4_angle = math.atan2(axis6y, axis6x)
                if k == 1:
                    a4_angle += math.pi
                    if(a4_angle > math.pi):
                        a4_angle -= 2 * math.pi
                
                a4_angle_wrapped = a4_angle + math.pi / 2
                while (a4_angle_wrapped >= math.pi):
                    a4_angle_wrapped -= 2 * math.pi
                while (a4_angle_wrapped < -math.pi):
                    a4_angle_wrapped += 2 * math.pi
                
                A4.append(a4_angle_wrapped)
                axis5_frame = rg.Plane(axis4_frame)
                axis5_frame.Rotate(a4_angle, axis4_frame.ZAxis)
                axis5_frame = rg.Plane(wrist, -axis5_frame.ZAxis, axis5_frame.XAxis)
                ok, axis6x, axis6y = axis5_frame.ClosestParameter(end_frame.Origin)
                a5_angle = math.atan2(axis6y, axis6x)
                A5.append(a5_angle)
                
                axis6_frame = rg.Plane(axis5_frame)
                axis6_frame.Rotate(a5_angle, axis5_frame.ZAxis)
                axis6_frame = rg.Plane(wrist, -axis6_frame.YAxis, axis6_frame.ZAxis)
                ok, endx, endy = axis6_frame.ClosestParameter(end_frame.PointAt(1, 0))
                a6_angle = math.atan2(endy, endx)
                A6.append(a6_angle)

    for i in range(8):
        A1[i] = A1[i] * -1
        A2[i] = A2[i] + math.radians(90)
        A3[i] = A3[i] - math.radians(90)
        A4[i] = A4[i] * -1
        A5[i] = A5[i]
        A6[i] = A6[i] * -1
    
    return A1, A2, A3, A4, A5, A6
