from __future__ import print_function
from compas_fabrication.fabrication.geometry import Frame, Transformation


class Robot(object):
    """
    This is the base class for all robots.
    It consists of:
    - a geometry (meshes)
    - a basis frame, the frame it resides, e.g. Frame.worldXY()
    - a transformation matrix to get coordinated represented in RCS
    - a transformation matrix to get coordinated represented in WCS
    - a tool, the end-effector
    - communication: e.g. delegated by a client instance
    - workspace: brep ?
 
    self.joint_values = [0,0,0,0,0,0]
    self.tcp_frame = tcp_frame
    self.tool0_frame = tool0_frame
    
    # transform world to robot origin
    self.T_W_R = rg.Transform.PlaneToPlane(Frame.worldXY, self.basis_frame)
    # transform robot to world
    self.T_R_W = rg.Transform.PlaneToPlane(self.basis_frame, Frame.worldXY)
    """
    
    def __init__(self, basis_frame = Frame.worldXY()):
        
        self.meshes = []
        self.basis_frame = None
        self.transformation_RCS_WCS = None
        self.transformation_WCS_RCS = None
        self.set_basis_frame(basis_frame)
        self.tool = None
        
        
        self.joint_angles = [0,0,0,0,0,0]
        
        self.tcp_frame = Frame.worldXY()
        self.tool0_frame = Frame.worldXY()
        
    
    def set_basis_frame(self, basis_frame):
        self.basis_frame = basis_frame
        # transformation matrix from world coordinate system to robot coordinate system
        self.transformation_RCS_WCS = Transformation.from_frame_to_frame(Frame.worldXY(), self.basis_frame)
        # transformation matrix from robot coordinate system to world coordinate system
        self.transformation_WCS_RCS = Transformation.from_frame_to_frame(self.basis_frame, Frame.worldXY())
    
    def set_tool(self, tool):
        self.tool = tool
        
    @property
    def tcp_frame(self):
        # read from tool
        if not self.tool:
            return self.tool0_frame
    
    def forward_kinematic(self, q):
        """
        Calculate the tcp frame according to the joint angles q.
        """
        # return tcp_frame_RCS
        raise NotImplementedError
    
    def inverse_kinematic(self, tcp_frame_RCS):
        """
        Calculate solutions (joint angles) according to the queried tcp frame
        (in RCS).
        """
    
    def get_frame_in_RCS(self, frame_WCS):
        """
        Transform the frame in world coordinate system (WCS) into a frame in 
        robot coordinate system (RCS), which is set by the robots' basis frame.
        """
        frame_RCS = frame_WCS.transform(self.transformation_WCS_RCS)
        return frame_RCS
    
            
    def get_tool0_frame_from_tcp_frame(self, frame_tcp):
        """
        Get the tool0 frame (frame at robot) from the tool frame (tcp),
        according to the set tool.
        
        """
        frame_tool0 = Frame()
        return frame_tool0
    

        """
        tool_plane_in_RCS = self.get_tool_plane_in_RCS(tp_WCS)

        T_TP_in_zero_W = rg.Transform.PlaneToPlane(self.tool.plane, rg.Plane.WorldXY)
        
        tcp_plane_in_RCS = rg.Plane.WorldXY
        tcp_plane_in_RCS.Transform(T_TP_in_zero_W)
        
        T_W_TP_in_RCS = rg.Transform.PlaneToPlane(rg.Plane.WorldXY, tool_plane_in_RCS)
        tcp_plane_in_RCS.Transform(T_W_TP_in_RCS)
        
        return tcp_plane_in_RCS
        """

    
class BaseConfiguration(object):
    """Represents the configuration of a robot based on its
    joint angle values and coordinates (position of external axis system, if any).

    Attributes:
        joint_values (:obj:`list` of :obj:`float`): Joint values expressed
            in degrees.
        coordinates (:obj:`list` of :obj:`float`): Position on the external axis
            system (if available).

    Examples:

        >>> from compas_fabrication.fabrication.robots import BaseConfiguration
        >>> config = BaseConfiguration.from_data({'joint_values': [90., 0., 0.]})
        >>> config.joint_values
        [90.0, 0.0, 0.0]


        >>> from compas_fabrication.fabrication.robots import BaseConfiguration
        >>> config = BaseConfiguration.from_data({'joint_values': [90., 0., 0., 0., 180., 45.],\
                                                 'coordinates': [8312.0]})
        >>> str(config)
        'joints: [90.0, 0.0, 0.0, 0.0, 180.0, 45.0], coordinates: [8312.0]'

    """
    def __init__(self):
        self.joint_values = None
        self.coordinates = None

    def __str__(self):
        return "joints: %s, coordinates: %s" % (self.joint_values, self.coordinates)

    @classmethod
    def from_joints(cls, joint_values):
        """Construct a configuration from joint values and coordinates.

        Args:
            joint_values (:obj:`list` of :obj:`float`): Joint values expressed
                in degrees.
            coordinates (:obj:`list` of :obj:`float`): Position on the external axis
                system (if available).

        Returns:
            Configuration: A :class:`.Configuration` instance.
        """
        return cls.from_joints_and_coordinates(joint_values, None)

    @classmethod
    def from_joints_and_coordinates(cls, joint_values, coordinates=None):
        """Construct a configuration from joint values and coordinates.

        Args:
            joint_values (:obj:`list` of :obj:`float`): Joint values expressed
                in degrees.
            coordinates (:obj:`list` of :obj:`float`): Position on the external axis
                system (if available).

        Returns:
            Configuration: A :class:`.Configuration` instance.
        """
        return cls.from_data({'joint_values': joint_values, 'coordinates': coordinates})

    @classmethod
    def from_data(cls, data):
        """Construct a configuration from its data representation.

        Args:
            data (`dict`): The data dictionary.

        Returns:
            Configuration: A :class:`.Configuration` instance.
        """
        config = cls()
        config.data = data
        return config

    def to_data(self):
        """Return the data dict that represents the configuration, and from which it can
        be reconstructed."""
        return self.data

    @property
    def data(self):
        """:obj:`dict` : The data representing the configuration.

        By assigning a data dict to this property, the current data of the configuration
        will be replaced by the data in the dict. The data getter and setter should
        always be used in combination with each other.
        """
        return {
            'joint_values': self.joint_values,
            'coordinates': self.coordinates
        }

    @data.setter
    def data(self, data):
        self.joint_values = data.get('joint_values') or None
        self.coordinates = data.get('coordinates') or None


class Pose(object):
    """Represents a robot pose described as a 4x4 transformation matrix.

    Attributes:
        values (:obj:`list` of :obj:`float`): list of 12 or 16 values representing a 4x4 matrix.
            If 12 values are provided, the last row is assumed to be ``[0 0 0 1]``.
    """
    def __init__(self):
        self.values = []

    def __str__(self):
        return "[%s, %s, %s, %s]" % (self.values[0:4], self.values[4:8], self.values[8:12], self.values[12:16])

    @classmethod
    def from_list(cls, list):
        """Construct a pose from a list of 12 or 16 :obj:`float` values.

        Args:
            list (:obj:`list` of :obj:`float`): list of 12 or 16 values representing a 4x4 matrix.

        Returns:
            Pose: A :class:`.Pose` instance.
        """
        return cls.from_data({'values': list})

    @classmethod
    def from_data(cls, data):
        """Construct a pose from its data representation.

        Args:
            data (`dict`): The data dictionary.

        Returns:
            Pose: A :class:`.Pose` instance.
        """
        pose = cls()
        pose.data = data
        return pose

    def to_data(self):
        """Return the data dict that represents the pose, and from which it can
        be reconstructed."""
        return self.data

    @property
    def data(self):
        """:obj:`dict` : The data representing the pose."""
        return {'values': self.values}

    @data.setter
    def data(self, data):
        values = data.get('values') or None

        if len(values) == 12:
            values.extend([0., 0., 0., 1.])
        elif len(values) != 16:
            raise ValueError('Expected 12 or 16 floats but got %d' % len(values))

        self.values = values


if __name__ == "__main__":
    
    basis_frame = Frame([-636.57, 370.83, 293.21], [0.00000, -0.54972, -0.83535], [0.92022, -0.32695, 0.21516])
    robot = Robot(basis_frame)
    T1 = robot.transformation_WCS_RCS
    T2 = robot.transformation_RCS_WCS
    print T1 * T2
    
    
    
    
