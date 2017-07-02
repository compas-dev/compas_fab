import math
from compas.geometry.elements import Point, Vector, Line
from compas.geometry.utilities import multiply_matrix_vector, multiply_matrices

__author__     = ['Romana Rust <rust@arch.ethz.ch>', ]


class Transformation():
    
    def __init__(self):
        """
        The Transformation class 
        Matrices describing affine transformation of the plane
        """
        self.matrix = [
            [1.0, 0.0, 0.0, 0.0], 
            [0.0, 1.0, 0.0, 0.0], 
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]]
    
    @classmethod
    def from_matrix(cls, matrix):
        T = cls()
        T.matrix = matrix[:]
    
    @classmethod
    def from_rotation(cls, rotation):
        pass
    
    @classmethod
    def from_translation(cls, translation):
        pass
    
    @classmethod
    def rotation(cls):
        R = Rotation.from_matrix(cls.matrix)
        R.matrix[0][3] = 0
        R.matrix[1][3] = 0
        R.matrix[2][3] = 0
    
    @classmethod
    def translation(cls):
        return Translation(cls.matrix[0][3], cls.matrix[1][3], cls.matrix[2][3])
    
    def __mul__(self, other):
        if type(other) == type(self): # rotation // translation // ..
            T = self.from_matrix(multiply_matrices(self.matrix, other.matrix))
            return T
        else:
            try:
                v = list(other)
                print "==>", v
                print "==>", type(v[0])
                if type(v[0]) == float:
                    print "float"
                    if len(v) == 3: # v = point or vector 
                        print v
                        v += [1.] # make homogeneous coordinates
                        print v
                        v = multiply_matrix_vector(self.matrix, v)
                        print v
                        return v[:3]
                else:
                    print "list"
                    if len(v) == 4 and len(v[0]) == 4: # v = also matrix!
                        m = multiply_matrices(self.matrix, v)
                        return m
                    else:
                        # could be a list of points that need to be transformed
                        xyz = zip(*v) # transpose matrix
                        xyz += [[1] * len(xyz[0])] # make homogeneous coordinates
                        xyz = multiply_matrices(self.matrix, xyz)
                        xyz = xyz[:3]
                        return zip(*xyz)
                        
            except:
                raise
                #raise "The type %s is not supported to be multiplied by this class." % type(other) # Input ERROr
            """
            elif type(other) == type(Vector([0,0,0])):
                v = list(other) + [1.] # Homogeneous vector
                v = multiply_matrix_vector(cls.matrix, v)
                return Vector(v[:3])
            elif str(type(other)) == str(type(Point([0,0,0]))):
                v = list(other) + [1.] # Homogeneous vector
                v = multiply_matrix_vector(cls.matrix, v)
                return Point(v[:3])
            elif str(type(other)) == str(type(Line([0,0,0],[0,0,0]))):
                start = list(other.start) + [1.] # Homogeneous vector
                start = multiply_matrix_vector(self.matrix, start)
                end = list(other.end) + [1.] # Homogeneous vector
                end = multiply_matrix_vector(self.matrix, end)
                return Line(start[:3], end[:3])
            elif type(other) == type([]):
                if len(other) != 3:
                    raise 'Rotation matrix shape is not compatible with vector length.'
                else:
                    v = other + [1.] # Homogeneous vector
                    v = multiply_matrix_vector(self.matrix, v)
                    return v[:3]
            else:
                print type(Point([0,0,0]))
                print type(other)
                print str(type(other)) == str(type(Point([0,0,0])))
                raise "The type %s is not supported to be multiplied by class Rotation." % type(other)
            """
        
    def __imul__(self, n):
        raise NotImplementedError
        
    
    def __getitem__(self, key):
        #raise KeyError
        raise NotImplementedError

    def __setitem__(self, key, value):
        raise NotImplementedError

    def __iter__(self):
        return iter(self.matrix)

    def __eq__(self, other):
        raise NotImplementedError

    def __add__(self, other):
        raise NotImplementedError

    def __iadd__(self, other):
        raise NotImplementedError

    def __sub__(self, other):
        raise NotImplementedError

    def __isub__(self, other):
        raise NotImplementedError

    
class Translation(Transformation):
    
    def __init__(self, translation):
        super(Transformation, self).__init__()
        self.matrix[0][3] = translation[0]
        self.matrix[1][3] = translation[1]
        self.matrix[2][3] = translation[2]

class Rotation(Transformation):
            
    @classmethod
    def from_basis_vectors(cls, xaxis, yaxis):
        if type(xaxis) == type([]): xaxis = Vector(xaxis)
        if type(yaxis) == type([]): yaxis = Vector(yaxis)
        xaxis.normalize()
        yaxis.normalize()
        zaxis = xaxis.cross(yaxis)
        R = cls()
        R.matrix[0][0], R.matrix[1][0], R.matrix[2][0] = list(xaxis)
        R.matrix[0][1], R.matrix[1][1], R.matrix[2][1] = list(yaxis)
        R.matrix[0][2], R.matrix[1][2], R.matrix[2][2] = list(zaxis)
        return R
    
    @classmethod
    def from_quaternion(cls, quaternion):
        """
        Calculate rotation matrix from quaternion.
        References Christoph Gohlke's implementation of quaternion_matrix(quaternion): 
        http://www.lfd.uci.edu/~gohlke/code/transformations.py.html
        """     
        q = quaternion
        n =  q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2 # dot product
        
        epsilon = 1.0e-15
        
        if n < epsilon:
            return cls()
        
        q = [v * math.sqrt(2.0 / n) for v in q]
        q = [[q[i]*q[j] for i in range(4)] for j in range(4)] # outer_product
        
        rotation = cls()
        rotation.matrix = [
            [1.0 - q[2][2] - q[3][3],       q[1][2] - q[3][0],       q[1][3] + q[2][0], 0.0],
            [      q[1][2] + q[3][0], 1.0 - q[1][1] - q[3][3],       q[2][3] - q[1][0], 0.0],
            [      q[1][3] - q[2][0],       q[2][3] + q[1][0], 1.0 - q[1][1] - q[2][2], 0.0],
            [                    0.0,                     0.0,                     0.0, 1.0]]
        return rotation
    
    @classmethod
    def from_axis_angle_vector(cls, axis_angle_vector):
        if type(axis_angle_vector) == type([]): axis_angle_vector = Vector(axis_angle_vector)
        angle = axis_angle_vector.length
        return cls.from_axis_and_angle(axis_angle_vector, angle)
    
    @classmethod
    def from_axis_and_angle(cls, axis, angle, point=None):
        """
        Calculate rotation matrix from axis-angle representation.
        References Christoph Gohlke's implementation of rotation_matrix(angle, direction, point=None): 
        http://www.lfd.uci.edu/~gohlke/code/transformations.py.html
        """
        if type(axis) == type([]): axis = Vector(axis)

        axis.normalize()

        sina = math.sin(angle)
        cosa = math.cos(angle)
        
        R = [[cosa, 0.0, 0.0], [0.0, cosa, 0.0], [0.0, 0.0, cosa]]
        
        outer_product = [[axis[i]*axis[j] * (1.0 - cosa) for i in range(3)] for j in range(3)]
        R = [[R[i][j] + outer_product[i][j] for i in range(3)] for j in range(3)]
    
        axis *= sina            
        m = [[    0.0, -axis[2],  axis[1]],
            [ axis[2],      0.0, -axis[0]],
            [-axis[1],  axis[0],      0.0]]
        
        rotation = cls()
        for i in range(3):
            for j in range(3):
                rotation.matrix[i][j] = R[i][j] + m[i][j]
                
                
        if point != None:
            
            t = Point(point) - Point(rotation * point)            
            rotation.matrix[0][3] = t.x
            rotation.matrix[1][3] = t.y
            rotation.matrix[2][3] = t.z
            return Transformation.from_matrix(rotation.matrix)
        else:
            return rotation 
        
    
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
    
    @property
    def axis_angle_vector(self):        
        """
        Calculate axis angle representation from rotation matrix.
        References Martin Baker's implementation at
        http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/index.htm
        More information can be found here:
        https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation
        """
        epsilon = 0.01 # margin to allow for rounding errors
        epsilon2 = 0.1 # margin to distinguish between 0 and 180 degrees
        
        m = self.matrix
        
        if ((math.fabs(m[0][1] - m[1][0]) < epsilon) and 
            (math.fabs(m[0][2] - m[2][0]) < epsilon) and 
            (math.fabs(m[1][2] - m[2][1]) < epsilon)):
            
            # Singularity found.
            # First check for identity matrix which must have + 1 for all terms in leading diagonal and zero in other terms
            if ((math.fabs(m[0][1] + m[1][0]) < epsilon2) and 
                (math.fabs(m[0][2] + m[2][0]) < epsilon2) and 
                (math.fabs(m[1][2] + m[2][1]) < epsilon2) and 
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
        return [angle * x, angle * y, angle * z]
        
    @property
    def euler_angles(self):
        raise NotImplementedError
    
    def __repr__(self):
        m = self.matrix
        s = "Rotation:\n"
        s += "%+.4f\t%+.4f\t%+.4f\t%+.4f\n" % (m[0][0], m[0][1], m[0][2], m[0][3])
        s += "%+.4f\t%+.4f\t%+.4f\t%+.4f\n" % (m[1][0], m[1][1], m[1][2], m[1][3])
        s += "%+.4f\t%+.4f\t%+.4f\t%+.4f\n" % (m[2][0], m[2][1], m[2][2], m[2][3])
        s += "%+.4f\t%+.4f\t%+.4f\t%+.4f\n" % (m[3][0], m[3][1], m[3][2], m[3][3])
        return s
    
    
if __name__ == "__main__":
    
    T = Transformation()
    print list(T)
    p = Point([1,2,3])
    print float
    print "==========================="
    print T * p
    l = Line([0,0,0],[1,0,0])
    print T * l
    print "==========================="
        
        