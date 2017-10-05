from __future__ import print_function
from compas_fab.fab.geometry import Frame, Transformation
from tool import Tool


class Robot(object):
    """Represents the base class for all robots.

    It consists of:
    - a model: meshes
    - a base: describes where the robot is attached to. This can be also a movable base: e.g. linear axis
    - a basis frame, the frame it resides, e.g. Frame.worldXY()
    - a transformation matrix to get coordinated represented in RCS
    - a transformation matrix to get coordinated represented in WCS
    - a tool, the end-effector
    - communication: e.g. delegated by a client instance
    - workspace: brep ?

    self.configuration = [0,0,0,0,0,0]
    self.tcp_frame = tcp_frame
    self.tool0_frame = tool0_frame

    # transform world to robot origin
    self.T_W_R = rg.Transform.PlaneToPlane(Frame.worldXY, self.basis_frame)
    # transform robot to world
    self.T_R_W = rg.Transform.PlaneToPlane(self.basis_frame, Frame.worldXY)
    """

    def __init__(self):

        self.model = []  # a list of meshes
        self.model_loaded = False
        self.basis_frame = None
        # move to UR !!!!
        self.transformation_RCS_WCS = None
        self.transformation_WCS_RCS = None
        self.set_base(Frame.worldXY())
        self.tool = Tool(Frame.worldXY())
        self.configuration = None
        self.tool0_frame = Frame.worldXY()

    def load_model(self):
        self.model_loaded = True

    def set_base(self, base_frame):
        # move to UR !!!!
        self.base_frame = base_frame
        # transformation matrix from world coordinate system to robot coordinate system
        self.transformation_RCS_WCS = Transformation.from_frame_to_frame(Frame.worldXY(), self.base_frame)
        # transformation matrix from robot coordinate system to world coordinate system
        self.transformation_WCS_RCS = Transformation.from_frame_to_frame(self.base_frame, Frame.worldXY())
        # modify joint axis !

    def set_tool(self, tool):
        self.tool = tool

    def get_robot_configuration(self):
        raise NotImplementedError

    @property
    def tcp_frame(self):
        return self.tool.tcp_frame

    @property
    def transformation_tool0_tcp(self):
        return self.tool.transformation_T0_T

    @property
    def transformation_tcp_tool0(self):
        return self.tool.transformation_T_T0

    def forward_kinematics(self, q):
        """Calculate the tcp frame according to the joint angles q.
        """
        raise NotImplementedError

    def inverse_kinematics(self, tcp_frame_RCS):
        """Calculate solutions (joint angles) according to the queried tcp frame
        (in RCS).
        """
        raise NotImplementedError

    def get_frame_in_RCS(self, frame_WCS):
        """Transform the frame in world coordinate system (WCS) into a frame in
        robot coordinate system (RCS), which is set by the robots' basis frame.
        """
        frame_RCS = frame_WCS.transform(self.transformation_WCS_RCS, copy=True)
        #frame_RCS = frame_WCS.transform(self.transformation_RCS_WCS)
        return frame_RCS

    def get_tool0_frame_from_tcp_frame(self, frame_tcp):
        """Get the tool0 frame (frame at robot) from the tool frame (tcp),
        according to the set tool.

        """
        T = Transformation.from_frame(frame_tcp)
        return Frame.from_transformation(T * self.transformation_tool0_tcp)


class BaseConfiguration(object):
    """Represents the configuration of a robot based on its
    joint angle values and external axes values (if any).

    Attributes:
        joint_values (:obj:`list` of :obj:`float`): Joint values expressed
            in degrees.
        external_axes (:obj:`list` of :obj:`float`): Position on the external axis
            system (if available).

    Examples:

        >>> from compas_fab.fab.robots import BaseConfiguration
        >>> config = BaseConfiguration.from_data({'joint_values': [90., 0., 0.]})
        >>> config.joint_values
        [90.0, 0.0, 0.0]


        >>> from compas_fab.fab.robots import BaseConfiguration
        >>> config = BaseConfiguration.from_data({'joint_values': [90., 0., 0., 0., 180., 45.],\
                                                 'external_axes': [8312.0]})
        >>> str(config)
        'joints: [90.0, 0.0, 0.0, 0.0, 180.0, 45.0], external_axes: [8312.0]'

    """

    def __init__(self):
        self.joint_values = None
        self.external_axes = None

    def __str__(self):
        return "joints: %s, external_axes: %s" % (self.joint_values, self.external_axes)

    @classmethod
    def from_joints(cls, joint_values):
        """Construct a configuration from joint values.

        Args:
            joint_values (:obj:`list` of :obj:`float`): Joint values expressed
                in degrees.

        Returns:
            Configuration: A :class:`.Configuration` instance.
        """
        return cls.from_joints_and_external_axes(joint_values, None)

    @classmethod
    def from_joints_and_external_axes(cls, joint_values, external_axes=None):
        """Construct a configuration from joint values and external axes values.

        Args:
            joint_values (:obj:`list` of :obj:`float`): Joint values expressed
                in degrees.
            external_axes (:obj:`list` of :obj:`float`): Position on the external axis
                system (if available).

        Returns:
            Configuration: A :class:`.Configuration` instance.
        """
        return cls.from_data({'joint_values': joint_values, 'external_axes': external_axes})

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
            'external_axes': self.external_axes
        }

    @data.setter
    def data(self, data):
        self.joint_values = data.get('joint_values') or None
        self.external_axes = data.get('external_axes') or None


# TODO this can merge with Frame, as Frame euler_axis, quaternion, ..
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
    base_frame = Frame([-636.57, 370.83, 293.21], [0.00000, -0.54972, -0.83535], [0.92022, -0.32695, 0.21516])
    robot = Robot()
    robot.set_base(base_frame)
    T1 = robot.transformation_WCS_RCS
    T2 = robot.transformation_RCS_WCS
    print(T1 * T2)
    print(robot.transformation_tcp_tool0)
