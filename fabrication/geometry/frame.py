from compas.geometry import cross_vectors
from compas.geometry.elements import Point, Vector
from compas_fabrication.fabrication.geometry.transformation import Rotation
from fabrication.geometry.transformation import Transformation

__author__     = ['Romana Rust <rust@arch.ethz.ch>', ]

    
class Frame():
    """The Frame consists of a point and and two orthonormal base vectors.
    
    Examples:
        frame = Frame.worldXY()
        frame = Frame.from_pose_quaternion([x, y, z, qw, qx, qy, qz])
        frame = Frame.from_pose_axis_angle_vector([x, y, z, ax, ay, az])
        frame = Frame.from_pose_euler_angles([x, y, z, a, b, c])
        frame = Frame.from_transformation(transformation)
        frame = Frame.from_rotation(rotation)
    """
    
    def __init__(self, point = Point([0, 0, 0]), xaxis=Vector([1, 0, 0]), yaxis=Vector([0, 1, 0])):
        
        if type(point) == type([]): 
            point = Point(point)
        if type(xaxis) == type([]): 
            xaxis = Vector(xaxis)
        if type(yaxis) == type([]): 
            yaxis = Vector(yaxis)
        
        xaxis.normalize()
        yaxis.normalize()
        
        self.point = point
        self.xaxis = xaxis
        self.yaxis = yaxis        
        self.yaxis = self.zaxis.cross(self.xaxis) # slight correction
        
    def copy(self):
        cls = type(self)
        cls.point = Point(list(self.point))
        cls.xaxis = Vector(list(self.xaxis))
        cls.yaxis = Vector(list(self.yaxis))
        return cls
        
    @classmethod        
    def worldXY(cls):
        frame = cls()
        frame.point = Point([0, 0, 0])
        frame.xaxis = Vector([1, 0, 0])
        frame.yaxis = Vector([0, 1, 0])
        return frame
    
    @classmethod        
    def worldZX(cls):
        frame = cls()
        frame.point = Point([0, 0, 0])
        frame.xaxis = Vector([0, 0, 1])
        frame.yaxis = Vector([1, 0, 0])
        return frame
    
    @classmethod        
    def worldYZ(cls):
        frame = cls()
        frame.point = Point([0, 0, 0])
        frame.xaxis = Vector([0, 1, 0])
        frame.yaxis = Vector([0, 0, 1])
        return frame
    
    @classmethod
    def from_quaternion(cls, quaternion):
        rotation = Rotation.from_quaternion(quaternion)
        frame = cls.from_rotation(rotation)
        return frame
    
    @classmethod
    def from_pose_quaternion(cls, pose_quaternion):
        x, y, z, qw, qx, qy, qz = pose_quaternion
        frame = cls.from_quaternion([qw, qx, qy, qz])
        frame.point = Point([x, y, z])
        return frame
    
    @classmethod 
    def from_axis_angle_vector(cls, axis_angle_vector):
        rotation = Rotation.from_axis_angle_vector(axis_angle_vector)
        frame = cls.from_rotation(rotation)
        return frame
    
    @classmethod
    def from_pose_axis_angle_vector(cls, pose_axis_angle_vector):
        x, y, z, ax, ay, az = pose_axis_angle_vector
        frame = cls.from_axis_angle_vector([ax, ay, az])
        frame.point = Point([x, y, z])
        return frame

    @classmethod 
    def from_euler_angles(cls, euler_angles):
        rotation = Rotation.from_euler_angles(euler_angles)
        frame = cls.from_rotation(rotation)
        return frame
    
    @classmethod
    def from_pose_euler_angles(cls, pose_euler_angles):
        x, y, z, a, b, c = pose_euler_angles
        frame = cls.from_euler_angles([a, b, c])
        frame.point = Point([x, y, z])
        return frame
    
    @classmethod
    def from_rotation(cls, rotation):
        xaxis, yaxis = rotation.basis_vectors
        return cls(Point([0,0,0]), xaxis, yaxis)
    
    @classmethod
    def from_transformation(cls, transformation):
        xaxis, yaxis = transformation.basis_vectors
        point = transformation.translation.vector
        return cls(point, xaxis, yaxis)
        
    @property
    def normal(self):
        normal = Vector(cross_vectors(self.xaxis, self.yaxis))
        normal.normalize()
        return normal
    
    @property
    def zaxis(self):
        return self.normal
        
    @property  
    def quaternion(self):
        rotation = Rotation.from_basis_vectors(self.xaxis, self.yaxis)
        return rotation.quaternion
    
    @property
    def pose_quaternion(self):
        """Get pose quaternion.
        
        Returns:
            list: coordinates and the rotation specified in quaternion, such as
            [x, y, z, qw, qx, qy, qz]
        """
        return list(self.point) + self.quaternion
            
    @property
    def axis_angle_vector(self):
        rotation = Rotation.from_basis_vectors(self.xaxis, self.yaxis)
        return rotation.axis_angle_vector
    
    @property
    def pose_axis_angle_vector(self):
        """Get pose axis angle.

        Returns:
            (list): coordinates and rotation specified in axis angle 
            representation, such as [x, y, z, ax, ay, az]
        """
        return list(self.point) + self.axis_angle_vector
    
    @property
    def euler_angles(self):
        R = Rotation.from_basis_vectors(self.xaxis, self.yaxis)
        return R.euler_angles
    
    @property
    def pose_euler_angles(self):
        """Get the frame represented as xyz coordinates with Euler angles.
        
        Returns:
            (list): [x, y, z, a, b, c]
        """
        return list(self.point) + self.euler_angles
    
    @property
    def rotation(self):
        return Rotation.from_basis_vectors(self.xaxis, self.yaxis)
    
    def transform(self, transformation):
        """Transforms the frame with transformation matrix.
        
        Returns:
            (Frame): the transformed frame.
        """
        point = Point(transformation.transform(self.point))
        xaxis = Vector(transformation.rotation().transform(self.xaxis))
        yaxis = Vector(transformation.rotation().transform(self.yaxis))
        return Frame(point, xaxis, yaxis)
            
    def __repr__(self):
        """
        TODO: Change this.
        """
        T = Transformation.from_frame(self)
        return str(T)
    

if __name__ == '__main__':
    
    """
    pose_quaternion =  [46.688110714374631, -1.4120551622885724, 49.438882686865952, 0.9222492523802307, -0.077292257754572713, 0.28255622706540073, 0.25227802504750946]
    print pose_quaternion
    frame = Frame.from_pose_quaternion(pose_quaternion)
    
    print frame.pose_quaternion
    
    
    R = rotation_matrix(q1, j1_vector, j1_end)
    print R
    """
    q1, j1_vector, j1_end = -2.02405833354, [-207.9183, -74.7322, 0.0], [-207.9183, -74.7322, 127.3]
    
    R = Rotation.from_axis_and_angle(j1_vector, q1)
    print R
    
    print "================"
    
    R = Rotation.from_axis_and_angle(Vector(j1_vector), q1, Point(j1_end))
    print R
    print R * R
    
    """
    print R * Vector([-207.9183, -74.7322, 0.0])
    print R * [-207.9183, -74.7322, 0.0]
    """
    xaxis = Vector([1, 0, 0])
    yaxis = Vector([0, 1, 0])
    zaxis = xaxis.cross(yaxis)
    
    print zaxis.cross(xaxis) # = yaxis
    
