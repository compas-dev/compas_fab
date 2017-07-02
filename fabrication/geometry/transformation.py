import math
from compas.geometry.elements import Point, Vector
from compas.geometry.utilities import multiply_matrix_vector, multiply_matrices
from compas.geometry import dot_vectors

__author__     = ['Romana Rust <rust@arch.ethz.ch>', ]


class Transformation(object):
    
    def __init__(self):
        """
        A ``Transformation`` object represents a 4x4 matrix describing an affine 
        transformation of the operations: rotation, translation, scale and 
        reflection.
        
        The __mul__ operator allows to concatenate a series of transformation 
        matrices.

        The class contains methods for converting rotation matrices to 
        axis-angle representations, Euler angles, and quaternions.
        """
        self.matrix = [
            [1.0, 0.0, 0.0, 0.0], 
            [0.0, 1.0, 0.0, 0.0], 
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]]
    
    @classmethod
    def from_matrix(cls, matrix):
        transformation = cls()
        transformation.matrix = matrix[:]
        return transformation
        
    def rotation(self):
        """ 
        Returns an instance of the Rotation class that just describes rotation.
        """
        return Rotation.from_matrix(self.matrix)
    
    def translation(self):
        """ 
        Return an instance of the Translation class that just describes translation.
        """
        return Translation.from_matrix(self.matrix)
    
    def __mul__(self, other):
        """
        The __mul__ operator performs a dot product with ``other``.
        ``other`` could be:
        - instance of Transformation class or derivative of Transformation class:
          this allows to concatenate a series of transformation matrices.
        - a Point, a Vector, xyz coordinates
        - a list of Points, Vectors or xyz coordinates
        - a 4x4 matrix (as list) describing a transformation
        """
        # multiply with instances of Transformation
        if self.__class__ == other.__class__:
            return Transformation.from_matrix(multiply_matrices(self.matrix, other.matrix))
        # multiply with instances of Rotation, Translation, etc.
        elif self.__class__ == other.__class__.__bases__[0] or self.__class__.__bases__[0] == other.__class__.__bases__[0]:
            return Transformation.from_matrix(multiply_matrices(self.matrix, other.matrix))
        else:
            try:
                v = list(other)
                # v = point, vector, xyz coordinates
                if type(v[0]) == float or type(v[0]) == int:
                    if len(v) == 3: 
                        v += [1.] # make homogeneous coordinates
                        v = multiply_matrix_vector(self.matrix, v)
                        return v[:3]
                    else:
                        raise Exception("The type %s is not supported to be multiplied by this class." % type(other))
                else:
                    # v = transformation matrix
                    if len(v) == 4 and len(v[0]) == 4:
                        return multiply_matrices(self.matrix, v)
                    # v = a list of xyz coordinates
                    else:
                        xyz = zip(*v) # transpose matrix
                        xyz += [[1] * len(xyz[0])] # make homogeneous coordinates
                        xyz = multiply_matrices(self.matrix, xyz)
                        xyz = xyz[:3]
                        return zip(*xyz) 
            except:
                raise Exception("The type %s is not supported to be multiplied by this class." % type(other))
        
    def __imul__(self, other):
        return Transformation.from_matrix(self.__mul__(other))
        
    def __getitem__(self, key):
        i, j = key
        return self.matrix[i][j]

    def __setitem__(self, key, value):
        i, j = key
        self.matrix[i][j] = value

    def __iter__(self):
        return iter(self.matrix)

    def __eq__(self, other):
        if self.__class__ == other.__class__:
            for i in range(4):
                for j in range(4):
                    if not self.matrix[i][j] == other.matrix[i][j]:
                        return False
            return True
        else:
            return False
    
    def __repr__(self):
        
        def format_number(number):
            return "%+.4f" % number if number < 0 else " %.4f" % number

        m = self.matrix
        s =  "[[%s, %s, %s, %s],\n" % tuple([format_number(n) for n in (m[0][0], m[0][1], m[0][2], m[0][3])])
        s += " [%s, %s, %s, %s],\n" % tuple([format_number(n) for n in (m[1][0], m[1][1], m[1][2], m[1][3])])
        s += " [%s, %s, %s, %s],\n" % tuple([format_number(n) for n in (m[2][0], m[2][1], m[2][2], m[2][3])])
        s += " [%s, %s, %s, %s]]"   % tuple([format_number(n) for n in (m[3][0], m[3][1], m[3][2], m[3][3])])
        return s
    
    def inverse(self):
        """
        Get the inverse Transformation.
        Attention: Works only if matrix is composed of translation and rotation, NOT scale or else.
        """
        inv_rotation = Rotation.from_matrix(self.matrix).inverse()
        trans = [-self.matrix[0][3], -self.matrix[1][3], -self.matrix[2][3]]
        trans = inv_rotation * trans
        
        cls = type(self)
        transformation = cls.from_matrix(inv_rotation.matrix)
        transformation.matrix[0][3] = trans[0]
        transformation.matrix[1][3] = trans[1]
        transformation.matrix[2][3] = trans[2]
        return transformation


class Rotation(Transformation):
    
    @classmethod
    def from_matrix(cls, matrix):
        # clean transformation matrix, so that it contains just rotation
        xaxis = [matrix[0][0], matrix[1][0], matrix[2][0]]
        yaxis = [matrix[0][1], matrix[1][1], matrix[2][1]]
        return cls.from_basis_vectors(xaxis, yaxis)
            
    @classmethod
    def from_basis_vectors(cls, xaxis, yaxis):
        """
        Create rotation matrix from basis vectors (= orthonormal row vectors).
        """
        if type(xaxis) == type([]): xaxis = Vector(xaxis)
        if type(yaxis) == type([]): yaxis = Vector(yaxis)
        xaxis.normalize()
        yaxis.normalize()
        zaxis = xaxis.cross(yaxis)
        rotation = cls()
        rotation.matrix[0][0], rotation.matrix[1][0], rotation.matrix[2][0] = list(xaxis)
        rotation.matrix[0][1], rotation.matrix[1][1], rotation.matrix[2][1] = list(yaxis)
        rotation.matrix[0][2], rotation.matrix[1][2], rotation.matrix[2][2] = list(zaxis)
        return rotation
    
    @classmethod
    def from_quaternion(cls, quaternion):
        """
        Create rotation matrix from quaternion.
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
        
        rotation = Rotation.from_matrix([
            [1.0 - q[2][2] - q[3][3],       q[1][2] - q[3][0],       q[1][3] + q[2][0], 0.0],
            [      q[1][2] + q[3][0], 1.0 - q[1][1] - q[3][3],       q[2][3] - q[1][0], 0.0],
            [      q[1][3] - q[2][0],       q[2][3] + q[1][0], 1.0 - q[1][1] - q[2][2], 0.0],
            [                    0.0,                     0.0,                     0.0, 1.0]])
        return rotation
    
    @classmethod
    def from_axis_angle_vector(cls, axis_angle_vector):
        """
        Create rotation matrix from axis-angle representation as vector.
        """ 
        if type(axis_angle_vector) == type([]): axis_angle_vector = Vector(axis_angle_vector)
        angle = axis_angle_vector.length
        return cls.from_axis_and_angle(axis_angle_vector, angle)
    
    @classmethod
    def from_axis_and_angle(cls, axis, angle, point=None):
        """
        Create rotation matrix from axis-angle representation.
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
            # rotation about axis, angle AND point includes also translation
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
    def axis_and_angle(self):        
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
                    return [0,0,0], 0
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
                                    
                return axis, angle # return 180 degree rotation
    
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

        return [x, y, z], angle
    
    @property
    def axis_angle_vector(self):
        axis, angle = self.axis_and_angle
        return [angle * axis[0], angle * axis[1], angle * axis[2]] 
    
    def inverse(self):
        """
        Get the inverse rotation.
        """
        inverse_rotation = Rotation.from_axis_angle_vector(Vector(self.axis_angle_vector) * -1)
        return inverse_rotation
        
    @property
    def euler_angles(self):
        raise NotImplementedError
    
class Translation(Transformation):
    
    
    @classmethod
    def from_vector(cls, vector):
        """
        Creates a matrix to translate by vector.
        """
        translation = cls()
        translation.matrix[0][3] = vector[0]
        translation.matrix[1][3] = vector[1]
        translation.matrix[2][3] = vector[2]
        return translation
    
    @classmethod
    def from_matrix(cls, matrix):
        # clean transformation matrix, so that it contains just translation
        return cls.from_vector([matrix[0][3], matrix[1][3], matrix[2][3]])

class Scale(Transformation):
    
    @classmethod
    def from_factor(cls, factor):
        """ 
        Creates a matrix to uniformly scale by factor. 
        """
        scale = cls()
        scale.matrix[0][0] = factor
        scale.matrix[1][1] = factor
        scale.matrix[2][2] = factor
        return scale
    
class Reflection(Transformation):
    
    @classmethod
    def from_point_and_normal(cls, point, normal):
        """
        Creates a matrix to mirror at plane, defined by point and normal vector.
        """
        reflection = cls()
        
        if type(normal) == type([]): normal = Vector(normal)
        normal.normalize()
        
        for i in range(3):
            for j in range(3):
                reflection.matrix[i][j] -= 2.0 * normal[i]*normal[j]
        
        for i in range(3):
            reflection.matrix[i][3] = 2 * dot_vectors(point, normal) * normal[i]
        return reflection
    
    @classmethod
    def from_plane(cls, plane):
        """
        Creates a matrix to mirror at plane.
        """
        return cls.from_point_and_normal(plane.point, plane.normal)
    
        
if __name__ == "__main__":
    
    pt = [19.961266434549813, 31.35370077557657, 0.0]
    normal = [-20.551039382110147, 22.135331787354541, 0.0]
    
    R = Reflection.from_point_and_normal(pt, normal)
    
    print "==============="
    
    rot = Rotation.from_matrix(R.matrix)
    print type(rot)
    print rot.__class__
    print ">>", rot.axis_and_angle
    
    print "==============="
    
    xaxis = [-0.1157, 0.3219, 0.9397]
    yaxis = [0.9411, 0.3382, 0.0000]
    vec = [40.4619, -112.5721, 170.8392]
    
    rotation = Rotation.from_basis_vectors(xaxis, yaxis)
    translation = Translation.from_vector(vec)
    transformation = translation * rotation
    print transformation
    print
    print transformation.inverse()
    print
    

        