from compas.geometry import cross_vectors
from compas.geometry.elements import Point, Vector, Plane
import math


__author__     = ['Romana Rust <rust@arch.ethz.ch>', ]


class Rotation():
    
    def __init__(self):
        """
        The Rotation class 
        """
        self.matrix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        
    @classmethod
    def from_basis_vectors(cls, xaxis, yaxis):
        xaxis.normalize()
        yaxis.normalize()
        zaxis = xaxis.cross(yaxis)
        R = cls()
        R.matrix = [list(xaxis), list(yaxis), list(zaxis)]
        return R
    
    @classmethod
    def from_quaternion(cls, quaternion):
        qw, qx, qy, qz = quaternion
        # make matrix
        raise NotImplementedError
    
    @classmethod
    def from_axis_angle(cls, axis_angle):
        ax, ay, az = axis_angle
        # make matrix ..
        raise NotImplementedError
    
    @classmethod
    def from_euler_angles(cls, euler_angles):
        #a, b, c = euler_angles
        raise NotImplementedError
    
    
    @property
    def quaternion(self):
        """
        Calculate quaternion from rotation matrix.
        References Martin Baker's implementation of matrix to quaternion: 
        http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/ 
        """
        
        m = self.matrix
        
        qw, qx, qy, qz = 0, 0, 0, 0
        trace = m[0][0] + m[1][1] + m[2][2]
        
        if trace > 0.0:
            s = (0.5 / math.sqrt(trace + 1.0))
            qw = 0.25 / s
            qx = (m[2][1] - m[1][2]) * s 
            qy = (m[0][2] - m[2][0]) * s
            qz = (m[1][0] - m[0][1]) * s
            
        elif ( (m[0][0] > m[1][1]) and (m[0][0] > m[2][2])):
            s = 2.0 * math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2])
            qw = (m[2][1] - m[1][2]) / s
            qx = 0.25 * s
            qy = (m[0][1] + m[1][0]) / s
            qz = (m[0][2] + m[2][0]) / s
            
        elif (m[1][1] > m[2][2] ):
            s = 2.0 * math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2])
            qw = (m[0][2] - m[2][0]) / s 
            qx = (m[0][1] + m[1][0]) / s
            qy = 0.25 * s
            qz = (m[1][2] + m[2][1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1])
            qw = (m[1][0] - m[0][1]) / s
            qx = (m[0][2] + m[2][0]) / s
            qy = (m[1][2] + m[2][1]) / s
            qz = 0.25 * s
            
        return [qw, qx, qy, qz]
    
    def __repr__(self):
        m = self.matrix
        s = "Rotation:\n"
        s += "%.4f\t%.4f\t%.4f\n" % (m[0][0], m[1][0], m[2][0])
        s += "%.4f\t%.4f\t%.4f\n" % (m[0][1], m[1][1], m[2][1])
        s += "%.4f\t%.4f\t%.4f\n" % (m[0][2], m[1][2], m[2][2])
        return s
    
    @property
    def axis_angle(self):        
        """
        Calculate axis angle representation from rotation matrix.
        References Martin Baker's implementation at
        http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/index.htm
        """
        epsilon = 0.01 # margin to allow for rounding errors
        epsilon2 = 0.1 # margin to distinguish between 0 and 180 degrees
        
        m = self.matrix
        
        if ((math.fabs(m[0][1] - m[1][0]) < epsilon) and \
            (math.fabs(m[0][2] - m[2][0]) < epsilon) and \
            (math.fabs(m[1][2] - m[2][1]) < epsilon)):
            
            # Singularity found.
            # First check for identity matrix which must have + 1 for all terms in leading diagonal and zero in other terms
            if ((math.fabs(m[0][1] + m[1][0]) < epsilon2) and \
                (math.fabs(m[0][2] + m[2][0]) < epsilon2) and \
                (math.fabs(m[1][2] + m[2][1]) < epsilon2) and \
                (math.fabs(m[0][0] + m[1][1] + m[2][2] - 3) < epsilon2)) :
                    # this singularity is identity matrix so angle = 0
                    return [0,0,0]
            else:
                # otherwise this singularity is angle = 180
                angle = math.pi
                xx = (m[0][0] + 1)/2
                yy = (m[1][1] + 1)/2
                zz = (m[2][2] + 1)/2
                xy = (m[0][1] + m[1][0])/4
                xz = (m[0][2] + m[2][0])/4
                yz = (m[1][2] + m[2][1])/4
                root_half = math.sqrt(0.5)
                if ((xx > yy) and (xx > zz)) : # m[0][0] is the largest diagonal term
                    if (xx < epsilon) :
                        axis = [0, root_half, root_half]
                    else:
                        x = math.sqrt(xx)
                        axis = [x, xy/x, xz/x]                    
                elif (yy > zz) : # m[11 is the largest diagonal term
                    if (yy < epsilon):
                        axis = [root_half, 0, root_half]
                    else :
                        y = math.sqrt(yy)
                        axis = [xy/y, y, yz/y]                        
                else : # m[2][2] is the largest diagonal term so base result on this
                    if (zz < epsilon) :
                        axis = [root_half, root_half, 0]
                    else :
                        z = math.sqrt(zz)
                        axis = [xz/z, yz/z, z]
                                    
                return [v * angle for v in axis] # return 180 degree rotation
    
        # as we have reached here there are no singularities so we can handle normally
        s = math.sqrt(\
            (m[2][1] - m[1][2])*(m[2][1] - m[1][2]) +
            (m[0][2] - m[2][0])*(m[0][2] - m[2][0]) +
            (m[1][0] - m[0][1])*(m[1][0] - m[0][1])) # used to normalize
        
        # prevent divide by zero][should not happen if matrix is orthogonal and should be
        # caught by singularity test above][but I've left it in just in case
        if (math.fabs(s) < 0.001): s = 1
        angle = math.acos((m[0][0] + m[1][1] + m[2][2] - 1)/2)
        
        x = (m[2][1] - m[1][2])/s
        y = (m[0][2] - m[2][0])/s
        z = (m[1][0] - m[0][1])/s
        return [v * angle for v in axis]
        
    @property
    def euler_angles(self):
        raise NotImplementedError
    
class Transformation():
    def __init__(self):
        pass
    
    @classmethod
    def identity(cls):
        return cls([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    
    @classmethod
    def from_translation(translation):
        x, y, z = translation
        return cls([[1.0, 0.0, 0.0, x], [0.0, 1.0, 0.0, y], [0.0, 0.0, 1.0, z], [0.0, 0.0, 0.0, 1.0]])
    
    @property
    def translation(self):
        m = self.matrix
        return [m[0][3], m[1][3], m[2][3]]
    
        

    
class Frame():
    
    def __init__(self, point = Point([0,0,0]), xaxis = Vector([1,0,0]), yaxis = Vector([0,1,0])):
        """
        The class "Frame" consists of a point and two base vectors (x- and y-axis).
        It contains methods to get the axis-angle representation (UR), 
        quaternions (ABB), or euler angles (Kuka) from this frame.
        the transformation matrix to and from another plane, and also contains some transformation methods.
        If no plane is given as an input, the origin of the plane is in world XY.
        If draw_geo is set to true: the axes can be visualized as lines (length = 100)
        
        quaternion definition: [qw, qx, qy, qz]
        angle_axis definition: [ax,ay,az] (angle = length of the vector, axis = vector)
        """
        
        self.point = point
        self.xaxis = xaxis
        self.yaxis = yaxis
    
    @classmethod        
    def worldXY(cls):
        frame = cls()
        frame.point = Point([0, 0, 0])
        frame.xaxis = Vector([1, 0, 0])
        frame.yaxis = Vector([0, 1, 0])
        return frame
    
    @classmethod        
    def worldXZ(cls):
        frame = cls()
        frame.point = Point([0, 0, 0])
        frame.xaxis = Vector([1, 0, 0])
        frame.yaxis = Vector([0, 0, 1])
        return frame
    
    @classmethod        
    def worldYZ(cls):
        frame = cls()
        frame.point = Point([0, 0, 0])
        frame.xaxis = Vector([0, 1, 0])
        frame.yaxis = Vector([0, 0, 1])
        return frame
    
    @classmethod
    def from_rhino_plane(cls, plane):
        frame = cls()
        frame.point = Point(list(plane.Origin))
        frame.xaxis = Vector(list(plane.XAxis))
        frame.yaxis = Vector(list(plane.YAxis))
        return frame
    
    @classmethod
    def from_quaternion(cls, quaternion):
        qw, qx, qy, qz = quaternion
        R = Rotation.from_quaternion([qw, qx, qy, qz])
        frame = cls()
        frame.xaxis = R.xaxis
        frame.yaxis = R.yaxis
        return frame
    
    @classmethod
    def from_pose_quaternion(cls, pose_quaternion):
        x, y, z, qw, qx, qy, qz = pose_quaternion
        frame = cls.from_quaternion([qw, qx, qy, qz])
        frame.point = Point([x, y, z])
        return frame
    
    @classmethod 
    def from_axis_angle(cls, axis_angle):
        R = Rotation.from_axis_angle(axis_angle)
        frame = cls()
        frame.xaxis = R.xaxis
        frame.yaxis = R.yaxis
        return frame
    
    @classmethod
    def from_pose_axis_angle(cls, pose_axis_angle):
        x, y, z, ax, ay, az = pose_axis_angle
        frame = cls.from_axis_angle([ax, ay, az])
        frame.point = Point([x, y, z])
        return frame

    @classmethod 
    def from_euler_angles(cls, euler_angles):
        R = Rotation.from_euler_angles(euler_angles)
        frame = cls()
        frame.xaxis = R.xaxis
        frame.yaxis = R.yaxis
        return frame
    
    @classmethod
    def from_pose_euler_angles(cls, pose_euler_angles):
        x, y, z, a, b, c = pose_euler_angles
        frame = cls.from_euler_angles([a, b, c])
        frame.point = Point([x, y, z])
        return frame
        
    @property
    def normal(self):
        return Vector(cross_vectors(self.xaxis, self.yaxis)).normalize()
        
    @property  
    def quaternion(self):
        R = Rotation.from_basis_vectors(self.xaxis, self.yaxis)
        return R.quaternion
    
    @property
    def pose_quaternion(self):
        """
        Returns a list with the rotation specified in quaternion, such as [x, y, z, qw, qx, qy, qz]
        """
        return list(self.point) + self.quaternion
            
    @property
    def axis_angle(self):
        R = Rotation.from_basis_vectors(self.xaxis, self.yaxis)
        return R.axis_angle
    
    @property
    def pose_axis_angle(self):
        """
        Returns a list with the rotation specified in axis_angle, such as [x, y, z, ax, ay, az]
        """
        return list(self.point) + self.axis_angle
    
    @property
    def euler_angles(self):
        R = Rotation.from_basis_vectors(self.xaxis, self.yaxis)
        return R.euler_angles
    
    @property
    def pose_euler_angles(self):
        """
        Returns a list with the rotation specified in euler angles, such as [x, y, z, a, b, c]
        """
        return list(self.point) + self.euler_angles
    
        


if __name__ == '__main__':
    p = Point([0, 0, 0])
    print p
    print p.x
    
    R = Rotation()
    print R
    what = R.axis_angle
    print what
    x = Vector([1, 2, 3])
    print x * 0
    v = x * 2
    a, b, c = v
    print a, b, c
    
    
    xaxis = Vector([4, 4, 4])
    yaxis = Vector([1, 5, 3])
    
    R = Rotation.from_basis_vectors(xaxis, yaxis)
    
    print R

    
    
    
    
