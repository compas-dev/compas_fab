from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import random

from compas.data import Data
from compas.geometry import Frame
from compas.geometry import Sphere
from compas.geometry import Transformation
from compas.robots import Configuration
from compas.robots import Joint
from compas.robots import RobotModel

from compas_fab.robots.constraints import Constraint
from compas_fab.robots.constraints import JointConstraint
from compas_fab.robots.constraints import OrientationConstraint
from compas_fab.robots.constraints import PositionConstraint

__all__ = [
    "Robot",
]


class Robot(Data):
    """Represents a robot.

    This class binds together several building blocks, such as the robot's
    descriptive model, its semantic information and an instance of a backend
    client into a cohesive programmable interface. This representation builds
    upon the model described in the class :class:`compas.robots.RobotModel` of
    the **COMPAS** framework.

    Attributes
    ----------
    model : :class:`RobotModel`
        The robot model, usually created from an URDF structure.
    artist : :class:`compas.robots.base_artist.BaseRobotModelArtist`
        Instance of the artist used to visualize the robot model. Defaults to ``None``.
    semantics : :class:`compas_fab.robots.RobotSemantics`
        The semantic model of the robot. Defaults to ``None``.
    client : :class:`compas_fab.backends.interfaces.ClientInterface`
        The backend client to use for communication,
        e.g. :class:`compas_fab.backends.RosClient`
    attributes : :obj:`dict`
        Named attributes related to the robot instance.
    attached_tools : :class:`compas_fab.robots.Tool`
        Dictionary mapping planning groups to the tool currently attached to them, if any.
    """

    def __init__(self, model=None, artist=None, semantics=None, client=None):
        super(Robot, self).__init__()
        # These attributes have to be initiated first, 
        # because they are used in the setters of the other attributes
        self._scale_factor = 1.0
        self._attached_tools = {}  # { planning_group_name: robots.tool.Tool }
        self._current_ik = {"request_id": None, "solutions": None}

        self.model = model
        self.artist = artist
        self.semantics = semantics
        self.client = client
        self.attributes = {}

    @property
    def data(self):
        data = {
            '_scale_factor': self._scale_factor,
            '_attached_tools': self._attached_tools,
            '_current_ik': self._current_ik,
            'model': self.model.data,
            # 'artist': self.artist.data if self.artist else None,
            # 'semantics': self.semantics.data if self.semantics else None,
            # 'client': self.client.data if self.client else None,
            'attributes': self.attributes,
        }
        return data

    @data.setter
    def data(self, data):
        self._scale_factor = data.get('_scale_factor', 1.0)
        self._attached_tools = data.get('_attached_tools', {})
        self._current_ik = data.get('_current_ik', {"request_id": None, "solutions": None})
        self.model = RobotModel.from_data(data['model'])
        # self.artist = data.get('artist', None)
        # self.semantics = data.get('semantics', None)
        # self.client = data.get('client', None)
        self.attributes = data.get('attributes', {})

    @property
    def artist(self):
        """:class:`compas.robots.base_artist.BaseRobotModelArtist`: Artist used to visualize robot model."""
        return self._artist

    @artist.setter
    def artist(self, artist):
        self._artist = artist
        if artist is None:
            return
        if len(self.model.joints) > 0 and len(self.model.links) > 0:
            self.scale(self._scale_factor)
            for tool in self.attached_tools.values():
                self.artist.attach_tool_model(tool.tool_model)

    @classmethod
    def basic(cls, name, joints=None, links=None, materials=None, **kwargs):
        """Create the most basic instance of a robot, based only on name.

        Parameters
        ----------
        name : :obj:`str`
            Name of the robot
        joints : :obj:`list` of :class:`compas.robots.Joint`, optional
            Robot's joints.
        links : :obj:`list` of :class:`compas.robots.Link`, optional
            Robot's links.
        materials : :obj:`list` of :class:`compas.robots.Material`, optional
            Material description of the robot.
        kwargs : :obj:`dict`
            Keyword arguments passed to the :class:`compas.robots.RobotModel`
            `attr` :obj:`dict`. Accessible from `Robot.model.attr`.


        Returns
        -------
        :class:`Robot`
            Newly created instance of a robot.

        Examples
        --------
        >>> robot = Robot.basic('A robot')
        >>> robot.name
        'A robot'
        """
        model = RobotModel(name, joints=joints or [], links=links or [], materials=materials or [], **kwargs)
        return cls(model, None)

    @property
    def name(self):
        """:obj:`str`: Name of the robot, as defined by its model.

        Examples
        --------
        >>> robot.name
        'ur5'
        """
        return self.model.name

    @property
    def group_names(self):
        """:obj:`list` of :obj:`str`: All planning groups of the robot.

        Examples
        --------
        >>> robot.group_names
        ['manipulator', 'endeffector']

        """
        self.ensure_semantics()
        return self.semantics.group_names

    @property
    def main_group_name(self):
        """:obj:`str`: Robot's main planning group."""
        self.ensure_semantics()
        return self.semantics.main_group_name

    @property
    def root_name(self):
        """:obj:`str`: Robot's root name."""
        return self.model.root.name

    @property
    def group_states(self):
        """:obj:`dict` of :obj:`dict`: All group states of the robot.

        Examples
        --------
        >>> sorted(robot.group_states['manipulator'].keys())
        ['home', 'up']

        """
        self.ensure_semantics()
        return self.semantics.group_states

    @property
    def attached_tools(self):
        """:obj:`dict` of :obj:`robot.Tool`: Maps planning group to the tool attached to it"""
        return self._attached_tools

    @property
    def attached_tool(self):
        """:obj:`robot.Tool`: For backwards compatibility. Returns the tool attached to the default group, or None."""
        return self._attached_tools.get(self.main_group_name, None)

    def get_end_effector_link_name(self, group=None):
        """Get the name of the robot's end effector link.

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the group. Defaults to the main planning group.

        Returns
        -------
        :obj:`str`

        Examples
        --------
        >>> robot.get_end_effector_link_name()
        'ee_link'
        """
        if not self.semantics:
            return self.model.get_end_effector_link_name()
        else:
            return self.semantics.get_end_effector_link_name(group)

    def get_end_effector_link(self, group=None):
        """Get the robot's end effector link.

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`compas.robots.Link`

        Examples
        --------
        >>> link = robot.get_end_effector_link()
        >>> link.name
        'ee_link'
        """
        name = self.get_end_effector_link_name(group)
        return self.model.get_link_by_name(name)

    def get_end_effector_frame(self, group=None, full_configuration=None):
        """Get the frame of the robot's end effector.

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.
        full_configuration : :class:`Configuration`, optional
            The robot's full configuration, i.e. values for all configurable
            joints of the entire robot. Defaults to the all-zero configuration.

        Returns
        -------
        :class:`compas.geometry.Frame`
        """
        if not full_configuration:
            full_configuration = self.zero_configuration()
        return self.model.forward_kinematics(full_configuration, link_name=self.get_end_effector_link_name(group))

    def get_base_link_name(self, group=None):
        """Get the name of the robot's base link.

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        str

        Examples
        --------
        >>> robot.get_base_link_name()
        'base_link'
        """
        if not self.semantics:
            return self.model.get_base_link_name()
        else:
            return self.semantics.get_base_link_name(group)

    def get_base_link(self, group=None):
        """Get the robot's base link.

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`compas.robots.Link`

        Examples
        --------
        >>> link = robot.get_base_link()
        >>> link.name
        'base_link'
        """
        name = self.get_base_link_name(group)
        return self.model.get_link_by_name(name)

    def get_base_frame(self, group=None, full_configuration=None):
        """Get the frame of the robot's base link, which is the robot's origin frame.

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.
        full_configuration : :class:`Configuration`, optional
            The robot's full configuration, i.e. values for all configurable
            joints of the entire robot. Defaults to the all-zero configuration.

        Returns
        -------
        :class:`compas.geometry.Frame`
        """
        if not full_configuration:
            full_configuration = self.zero_configuration()
        return self.model.forward_kinematics(full_configuration, link_name=self.get_base_link_name(group))

    def get_link_names(self, group=None):
        """Get the names of the links in the kinematic chain.

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :obj:`list` of :obj:`str`

        Examples
        --------
        >>> robot.get_link_names('manipulator')
        ['base_link', 'shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'ee_link']
        """
        base_link_name = self.get_base_link_name(group)
        ee_link_name = self.get_end_effector_link_name(group)
        link_names = []
        for link in self.model.iter_link_chain(base_link_name, ee_link_name):
            link_names.append(link.name)
        return link_names

    def get_link_names_with_collision_geometry(self):
        """Get the names of the links with collision geometry.

        Returns
        -------
        :obj:`list` of :obj:`str`

        Examples
        --------
        >>> robot.get_link_names_with_collision_geometry()
        ['base_link', 'shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'ee_link']
        """
        return [link.name for link in self.model.iter_links() if link.collision]

    def get_configurable_joints(self, group=None):
        """Get the robot's configurable joints.

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :obj:`list` of :class:`compas.robots.Joint`

        Note
        ----
        If semantics is set and no group is passed, it returns all configurable
        joints of all groups.

        Examples
        --------
        >>> joints = robot.get_configurable_joints('manipulator')
        >>> [j.name for j in joints]
        ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        """
        if self.semantics:
            if group:
                return self.semantics.get_configurable_joints(group)
            else:
                return self.semantics.get_all_configurable_joints()
        else:
            return self.model.get_configurable_joints()

    def get_joint_types_by_names(self, names):
        """Get a list of joint types given a list of joint names.

        Parameters
        ----------
        names : :obj:`list` of :obj:`str`
            The names of the joints.

        Returns
        -------
        :obj:`list` of :attr:`compas.robots.Joint.SUPPORTED_TYPES`
            List of joint types.
        """
        return self.model.get_joint_types_by_names(names)

    def get_joint_by_name(self, name):
        """RGet the joint in the robot model matching the given name.

        Parameters
        ----------
        name : :obj:`str`
            The name of the joint.

        Returns
        -------
        :class:`compas.robots.Joint`
        """
        return self.model.get_joint_by_name(name)

    def get_configurable_joint_names(self, group=None):
        """Get the configurable joint names.

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :obj:`list` of :obj:`str`

        Note
        ----
        If semantics is set and no group is passed, it returns all configurable
        joints of all groups.

        Examples
        --------
        >>> robot.get_configurable_joint_names('manipulator')
        ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', \
        'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        """
        configurable_joints = self.get_configurable_joints(group)
        return [j.name for j in configurable_joints]

    def get_configurable_joint_types(self, group=None):
        """Get the configurable joint types.

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :obj:`list` of :attr:`compas.robots.Joint.SUPPORTED_TYPES`

        Note
        ----
        If :attr:`semantics` is set and no group is passed, it returns all
        configurable joint types of all groups.

        Examples
        --------
        >>> robot.get_configurable_joint_types('manipulator')
        [0, 0, 0, 0, 0, 0]
        """
        configurable_joints = self.get_configurable_joints(group)
        return [j.type for j in configurable_joints]

    def get_attached_tool_collision_meshes(self):
        """Returns a list of all attached collisions meshes of each of the attached tools, if any.

        Returns
        -------
        A list of lists.
        List[List[compas_fab.robots.planning_scene.CollisionMesh]]
        """
        return [tool.attached_collision_meshes for tool in self.attached_tools.values()]

    # ==========================================================================
    # configurations
    # ==========================================================================

    def zero_configuration(self, group=None):
        """Get the zero joint configuration.

        If zero is out of joint limits ``(upper, lower)`` then
        ``(upper + lower) / 2`` is used as joint value.

        Examples
        --------
        >>> robot.zero_configuration('manipulator')
        Configuration((0.000, 0.000, 0.000, 0.000, 0.000, 0.000), (0, 0, 0, 0, 0, 0), \
            ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'))
        """
        values = []
        joint_names = []
        joint_types = []
        for joint in self.get_configurable_joints(group):
            if joint.limit and not (0 <= joint.limit.upper and 0 >= joint.limit.lower):
                values.append((joint.limit.upper + joint.limit.lower) / 2.0)
            else:
                values.append(0)
            joint_names.append(joint.name)
            joint_types.append(joint.type)
        return Configuration(values, joint_types, joint_names)

    def random_configuration(self, group=None):
        """Get a random configuration.

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`compas.robots.Configuration`

        Note
        ----
        No collision checking is involved, the configuration may be invalid.
        """
        configurable_joints = self.get_configurable_joints(group)
        values = []
        for joint in configurable_joints:
            if joint.limit:
                values.append(joint.limit.lower + (joint.limit.upper - joint.limit.lower) * random.random())
            else:
                values.append(0)
        joint_names = self.get_configurable_joint_names(group)
        joint_types = self.get_joint_types_by_names(joint_names)
        return Configuration(values, joint_types, joint_names)

    def get_group_configuration(self, group, full_configuration):
        """Get the group's configuration.

        Parameters
        ----------
        group : :obj:`str`
            The name of the planning group.
        full_configuration : :class:`Configuration`
            The configuration for all configurable joints of the robot.

        Returns
        -------
        :class:`Configuration`
            The configuration of the group.
        """
        full_configuration = self._check_full_configuration_and_scale(full_configuration)[
            0
        ]  # adds joint_names to full_configuration and makes copy
        group_joint_names = self.get_configurable_joint_names(group)
        values = [full_configuration[name] for name in group_joint_names]
        return Configuration(values, self.get_configurable_joint_types(group), group_joint_names)

    def merge_group_with_full_configuration(self, group_configuration, full_configuration, group):
        """Get the robot's full configuration by merging a group's configuration with a full configuration.
        The group configuration takes precedence over the full configuration in
        case a joint value is present in both.

        Parameters
        ----------
        group_configuration : :class:`Configuration`
            The configuration for one of the robot's planning groups.
        full_configuration : :class:`Configuration`
            The configuration for all configurable joints of the robot.
        group : :obj:`str`
            The name of the planning group.

        Returns
        -------
        :class:`Configuration`
            A full configuration with values for all configurable joints.

        Raises
        ------
        :exc:`ValueError`
            If the `full_configuration` does not specify positions for all
            configurable joints, or if the `group_configuration` does not
            specify positions for all configurable joints of the given group.
        """
        if not group_configuration.joint_names:
            group_configuration.joint_names = self.get_configurable_joint_names(group)

        full_configuration = self._check_full_configuration_and_scale(full_configuration)[
            0
        ]  # adds joint_names to full_configuration and makes copy

        full_configuration = full_configuration.merged(group_configuration)
        return full_configuration

    def get_group_names_from_link_name(self, link_name):
        """Get the names of the groups `link_name` belongs to.

        Parameters
        ----------
        link_name : :obj:`str`
            The name of a link

        Returns
        -------
        :obj:`list` of :obj:`str`
           A list of group names.
        """
        group_names = []
        for group in self.group_names:
            if link_name in self.get_link_names(group):
                group_names.append(group)
        return group_names

    def get_position_by_joint_name(self, configuration, joint_name, group=None):
        """Get the position of named joint in given configuration.

        Parameters
        ----------
        configuration : :class:`Configuration`
            The configuration of the configurable joints.
        joint_name : :obj:`str`
            Name of joint.
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :obj:`float`
            Joint position for the given joint.

        Raises
        ------
        :exc:`ValueError`
            If the number of joints in the `configuration` parameter does not
            match the configurable joints of the given `group`.
        """
        names = self.get_configurable_joint_names(group)
        if len(names) != len(configuration.joint_values):
            raise ValueError("Please pass a configuration with {} joint_values or specify group".format(len(names)))
        return configuration.joint_values[names.index(joint_name)]

    def _check_full_configuration_and_scale(self, full_configuration=None):
        """Either create a full configuration or check if the passed full configuration is valid.

        Parameters
        ----------
        full_configuration : :class:`Configuration`, optional
            The full configuration of the whole robot, including values for all configurable joints.

        Returns
        -------
        (:class:`Configuration`, :class:`Configuration`)
            The full configuration and the scaled full configuration
        """
        if not full_configuration:
            configuration = self.zero_configuration()  # with joint_names
        else:
            joint_names = self.get_configurable_joint_names()  # full configuration
            # full_configuration might have passive joints specified as well, we allow this.
            if len(joint_names) > len(full_configuration.joint_values):
                raise ValueError(
                    "Please pass a configuration with {} values, for all configurable joints of the robot.".format(
                        len(joint_names)
                    )
                )
            configuration = full_configuration.copy()
            if not configuration.joint_names:
                configuration.joint_names = joint_names
        return configuration, configuration.scaled(1.0 / self.scale_factor)

    def get_configuration_from_group_state(self, group, group_state):
        """Get a ``Configuration`` from a group's group state.

        Parameters
        ----------
        group : :obj:`str`
            The name of the planning group.
        group_state : :obj:`str
            The name of the ``group_state``.

        Returns
        -------
        :class:`Configuration`
            The configuration specified by the ``group_state``.
        """
        joint_dict = self.group_states[group][group_state]
        group_joint_names = self.get_configurable_joint_names(group)
        group_joint_types = self.get_configurable_joint_types(group)
        values = [joint_dict[name] for name in group_joint_names]
        return Configuration(values, group_joint_types, group_joint_names)

    # ==========================================================================
    # transformations, coordinate frames
    # ==========================================================================

    def transformation_RCF_WCF(self, group=None):
        """Get the transformation from the robot's coordinate system (RCF) to the world coordinate system (WCF).

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`compas.geometry.Transformation`

        """
        base_frame = self.get_base_frame(group)
        return Transformation.from_change_of_basis(base_frame, Frame.worldXY())

    def transformation_WCF_RCF(self, group=None):
        """Get the transformation from the world coordinate system (WCF) to the robot's coordinate system (RCF).

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`compas.geometry.Transformation`

        """
        base_frame = self.get_base_frame(group)
        return Transformation.from_change_of_basis(Frame.worldXY(), base_frame)

    def set_RCF(self, robot_coordinate_frame, group=None):
        """Move the origin frame of the robot to the robot_coordinate_frame.

        Raises
        ------
        :exc:`NotImplementedError`
            Not implemented yet.
        """
        # TODO: must be applied to the model, so that base_frame is RCF
        # Problem: check if conversion wcf/rcf still works with backend
        raise NotImplementedError

    def get_RCF(self, group=None):
        """Get the origin frame of the robot.

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`compas.geometry.Frame`
            Origin frame of the robot.
        """
        return self.get_base_frame(group)

    def to_local_coordinates(self, frame_WCF, group=None):
        """Represent a frame from the world coordinate system (WCF) in the robot's coordinate system (RCF).

        Parameters
        ----------
        frame_WCF : :class:`compas.geometry.Frame`
            A frame in the world coordinate frame.
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`compas.geometry.Frame`
            A frame in the robot's coordinate frame.

        Examples
        --------
        >>> frame_WCF = Frame([-0.363, 0.003, -0.147], [0.388, -0.351, -0.852], [0.276, 0.926, -0.256])
        >>> frame_RCF = robot.to_local_coordinates(frame_WCF)
        >>> frame_RCF
        Frame(Point(-0.363, 0.003, -0.147), Vector(0.388, -0.351, -0.852), Vector(0.276, 0.926, -0.256))
        """
        frame_RCF = frame_WCF.transformed(self.transformation_WCF_RCF(group))
        return frame_RCF

    def to_world_coordinates(self, frame_RCF, group=None):
        """Represent a frame from the robot's coordinate system (RCF) in the world coordinate system (WCF).

        Parameters
        ----------
        frame_RCF : :class:`compas.geometry.Frame`
            A frame in the robot's coordinate frame.
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`compas.geometry.Frame`
            A frame in the world coordinate frame.

        Examples
        --------
        >>> frame_RCF = Frame([-0.363, 0.003, -0.147], [0.388, -0.351, -0.852], [0.276, 0.926, -0.256])
        >>> frame_WCF = robot.to_world_coordinates(frame_RCF)
        >>> frame_WCF
        Frame(Point(-0.363, 0.003, -0.147), Vector(0.388, -0.351, -0.852), Vector(0.276, 0.926, -0.256))
        """
        frame_WCF = frame_RCF.transformed(self.transformation_RCF_WCF(group))
        return frame_WCF

    def _get_attached_tool_for_group(self, group_name=None):
        """Get the tool attached to the given planning group. Group name defaults to main_group_name.
        Raises ValueError if group name is unknown or there is no tool currently attached to it"""
        group = group_name or self.main_group_name
        if group not in self.attached_tools:
            raise ValueError("No tool attached to group {}".format(group))

        return self.attached_tools[group]

    def from_tcf_to_t0cf(self, frames_tcf, group=None):
        """Convert a list of frames at the robot's tool tip (tcf frame) to frames at the robot's flange (tool0 frame) using the attached tool.

        Parameters
        ----------
        frames_tcf : :obj:`list` of :class:`compas.geometry.Frame`
            Frames (in WCF) at the robot's tool tip (tcf).

        group : :obj:`str`, optional
            The planning group whose tool to use. Defaults to the main
            planning group.

        Returns
        -------
        :obj:`list` of :class:`compas.geometry.Frame`
            Frames (in WCF) at the robot's flange (tool0).

        Raises
        ------
        Exception
            If the attached tool is not set.

        Examples
        --------
        >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
        >>> frame = Frame([0.14, 0, 0], [0, 1, 0], [0, 0, 1])
        >>> robot.attach_tool(Tool(mesh, frame))
        >>> frames_tcf = [Frame((-0.309, -0.046, -0.266), (0.276, 0.926, -0.256), (0.879, -0.136, 0.456))]
        >>> robot.from_tcf_to_t0cf(frames_tcf)
        [Frame(Point(-0.363, 0.003, -0.147), Vector(0.388, -0.351, -0.852), Vector(0.276, 0.926, -0.256))]
        >>> robot.from_tcf_to_t0cf(frames_tcf, group=robot.main_group_name)
        [Frame(Point(-0.363, 0.003, -0.147), Vector(0.388, -0.351, -0.852), Vector(0.276, 0.926, -0.256))]
        """
        tool = self._get_attached_tool_for_group(group_name=group)
        return tool.from_tcf_to_t0cf(frames_tcf)

    def from_t0cf_to_tcf(self, frames_t0cf, group=None):
        """Convert frames at the robot's flange (tool0 frame) to frames at the robot's tool tip (tcf frame) using the attached tool.

        Parameters
        ----------
        frames_t0cf : :obj:`list` of :class:`compas.geometry.Frame`
            Frames (in WCF) at the robot's flange (tool0).

        group : :obj:`str`, optional
            The planning group to attach this tool to. Defaults to the main
            planning group.

        Returns
        -------
        :obj:`list` of :class:`compas.geometry.Frame`
            Frames (in WCF) at the robot's tool tip (tcf).

        Raises
        ------
        :exc:`Exception`
            If the robot has no attached tool defined.

        Examples
        --------
        >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
        >>> frame = Frame([0.14, 0, 0], [0, 1, 0], [0, 0, 1])
        >>> robot.attach_tool(Tool(mesh, frame))
        >>> frames_t0cf = [Frame((-0.363, 0.003, -0.147), (0.388, -0.351, -0.852), (0.276, 0.926, -0.256))]
        >>> robot.from_t0cf_to_tcf(frames_t0cf)
        [Frame(Point(-0.309, -0.046, -0.266), Vector(0.276, 0.926, -0.256), Vector(0.879, -0.136, 0.456))]
        >>> robot.from_t0cf_to_tcf(frames_t0cf, group=robot.main_group_name)
        [Frame(Point(-0.309, -0.046, -0.266), Vector(0.276, 0.926, -0.256), Vector(0.879, -0.136, 0.456))]
        """
        tool = self._get_attached_tool_for_group(group_name=group)
        return tool.from_t0cf_to_tcf(frames_t0cf)

    def attach_tool(self, tool, group=None, touch_links=None):
        """Attach a tool to the robot independently of the model definition.

        Parameters
        ----------
        tool : :class:`Tool`
            The tool that should be attached to the robot's flange.
        group : :obj:`str`, optional
            The planning group to attach this tool to. Defaults to the main
            planning group.
        touch_links : :obj:`list` of :obj:`str`, optional
            A list of link names the end-effector is allowed to touch. Defaults
            to the end-effector link.

        Returns
        -------
        ``None``

        See Also
        --------
        * :meth:`detach_tool`

        Examples
        --------
        >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
        >>> frame = Frame([0.14, 0, 0], [0, 1, 0], [0, 0, 1])
        >>> tool = Tool(mesh, frame)
        >>> robot.attach_tool(tool)
        """
        group = group or self.main_group_name
        if group not in self.semantics.group_names:
            raise ValueError("No such group: {}".format(group))

        if not tool.link_name:
            tool.link_name = self.get_end_effector_link_name(group)
        tool.update_touch_links(touch_links)
        self.attached_tools[group] = tool

        if self.artist:
            self.artist.attach_tool_model(tool.tool_model)

    def detach_tool(self, group=None):
        """Detach the attached tool.

        Parameters
        ----------
        group : :obj:`str`, optional
            The planning group to attach this tool to. Defaults to the main
            planning group.

        See Also
        --------
        * :meth:`attach_tool`
        """
        group = group or self.main_group_name
        if group not in self.attached_tools:
            raise ValueError("No tool attached to group {}".format(group))

        tool_to_remove = self.attached_tools[group]
        if self.artist and tool_to_remove:
            self.artist.detach_tool_model(tool_to_remove.tool_model)
        self.attached_tools.pop(group)

    # ==========================================================================
    # checks
    # ==========================================================================

    def ensure_client(self):
        """Check if the client is set.
        Raises
        ------
        :exc:`Exception`
            If :attr:`client` is not set
        """
        if not self.client:
            raise Exception("This method is only callable once a client is assigned.")

    def ensure_semantics(self):
        """Check if semantics is set.
        Raises
        ------
        :exc:`Exception`
            If :attr:`semantics` is not set.
        """
        if not self.semantics:
            raise Exception("This method is only callable once a semantic model is assigned.")

    def ensure_geometry(self):
        """Check if the model's geometry has been loaded.
        Raises
        ------
        :exc:`Exception`
            If geometry has not been loaded.
        """
        self.model.ensure_geometry()

    # ==========================================================================
    # constraints
    # ==========================================================================

    def orientation_constraint_from_frame(self, frame_WCF, tolerances_axes, group=None, use_attached_tool_frame=True):
        r"""Create an orientation constraint from a frame on the group's end-effector link.

        Parameters
        ----------
        frame_WCF: :class:`compas.geometry.Frame`
            The frame from which we create the orientation constraint.
        tolerances_axes: :obj:`list` of :obj:`float`
            Error tolerances t\ :sub:`i` for each of the frame's axes in
            radians. If only one value is passed it will be uses for all 3 axes.
        group: :obj:`str`, optional
            The planning group for which we specify the constraint. Defaults to
            the robot's main planning group.
        use_attached_tool_frame : :obj:`bool`, optional
            If ``True`` and there is a tool attached to the planning group, it will use its TCF
            instead of the T0CF to create the constraints. Defaults to ``True``.

        Returns
        -------
        :class:`OrientationConstraint`

        Raises
        ------
        :exc:`ValueError`
            If tolerance axes given are not one or three values.

        Notes
        -----
        The rotation tolerance for an axis is defined by the other vector
        component values for rotation around corresponding axis.
        If you specify the tolerances_axes vector with ``[0.01, 0.01, 6.3]``, it
        means that the frame's x-axis and y-axis are allowed to rotate about the
        z-axis by an angle of 6.3 radians, whereas the z-axis would only rotate
        by 0.01.


        Examples
        --------
        >>> frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
        >>> tolerances_axes = [math.radians(1)] * 3
        >>> group = robot.main_group_name
        >>> robot.orientation_constraint_from_frame(frame, tolerances_axes, group=group)
        OrientationConstraint('ee_link', [0.5, 0.5, 0.5, 0.5], [0.017453292519943295, 0.017453292519943295, 0.017453292519943295], 1.0)
        """

        attached_tool = self.attached_tools.get(group)
        if use_attached_tool_frame and attached_tool:
            frame_WCF = self.from_tcf_to_t0cf([frame_WCF], group)[0]

        ee_link = self.get_end_effector_link_name(group)

        tolerances_axes = list(tolerances_axes)
        if len(tolerances_axes) == 1:
            tolerances_axes *= 3
        elif len(tolerances_axes) != 3:
            raise ValueError("Must give either one or 3 values")

        return OrientationConstraint(ee_link, frame_WCF.quaternion, tolerances_axes)

    def position_constraint_from_frame(self, frame_WCF, tolerance_position, group=None, use_attached_tool_frame=True):
        """Create a position constraint from a frame on the group's end-effector link.

        Parameters
        ----------
        frame_WCF : :class:`compas.geometry.Frame`
            The frame from which we create position and orientation constraints.
        tolerance_position : :obj:`float`
            The allowed tolerance to the frame's position (defined in the
            robot's units).
        group: :obj:`str`, optional
            The planning group for which we specify the constraint. Defaults to
            the robot's main planning group.
        use_attached_tool_frame : :obj:`bool`, optional
            If ``True`` and there is a tool attached to the planning group, it will use its TCF
            instead of the T0CF to create the constraints. Defaults to ``True``.

        Returns
        -------
        :class:`PositionConstraint`

        See Also
        --------
        * :meth:`PositionConstraint.from_box`
        * :meth:`PositionConstraint.from_mesh`
        * :meth:`PositionConstraint.from_sphere`

        Notes
        -----
        The rotation tolerance for an axis is defined by the other vector
        component values for rotation around corresponding axis.
        If you specify the tolerances_axes vector with ``[0.01, 0.01, 6.3]``, it
        means that the frame's x-axis and y-axis are allowed to rotate about the
        z-axis by an angle of 6.3 radians, whereas the z-axis would only rotate
        by 0.01.

        Examples
        --------
        >>> frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
        >>> tolerance_position = 0.001
        >>> robot.position_constraint_from_frame(frame, tolerance_position)
        PositionConstraint('ee_link', BoundingVolume(2, Sphere(Point(0.400, 0.300, 0.400), 0.001)), 1.0)
        """

        attached_tool = self.attached_tools.get(group)
        if use_attached_tool_frame and attached_tool:
            frame_WCF = self.from_tcf_to_t0cf([frame_WCF], group)[0]

        ee_link = self.get_end_effector_link_name(group)
        sphere = Sphere(frame_WCF.point, tolerance_position)
        return PositionConstraint.from_sphere(ee_link, sphere)

    def constraints_from_frame(
        self, frame_WCF, tolerance_position, tolerances_axes, group=None, use_attached_tool_frame=True
    ):
        r"""Create a position and an orientation constraint from a frame calculated for the group's end-effector link.

        Parameters
        ----------
        frame_WCF: :class:`compas.geometry.Frame`
            The frame from which we create position and orientation constraints.
        tolerance_position: :obj:`float`
            The allowed tolerance to the frame's position (defined in the
            robot's units).
        tolerances_axes: :obj:`list` of :obj:`float`
            Error tolerances t\ :sub:`i` for each of the frame's axes in
            radians. If only one value is passed it will be uses for all 3 axes.
        group: :obj:`str`, optional
            The planning group for which we specify the constraint. Defaults to
            the robot's main planning group.
        use_attached_tool_frame : :obj:`bool`, optional
            If ``True`` and there is a tool attached to the planning group, it will use its TCF
            instead of the T0CF to create the constraints. Defaults to ``True``.

        Returns
        -------
        :obj:`list` of :class:`Constraint`

        See Also
        --------
        * :meth:`PositionConstraint.from_box`
        * :meth:`PositionConstraint.from_mesh`
        * :meth:`PositionConstraint.from_sphere`
        * :meth:`orientation_constraint_from_frame`

        Notes
        -----
        The rotation tolerance for an axis is defined by the other vector
        component values for rotation around corresponding axis.
        If you specify the tolerances_axes vector with ``[0.01, 0.01, 6.3]``, it
        means that the frame's x-axis and y-axis are allowed to rotate about the
        z-axis by an angle of 6.3 radians, whereas the z-axis would only rotate
        by 0.01.


        Examples
        --------
        >>> frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
        >>> tolerance_position = 0.001
        >>> tolerances_axes = [math.radians(1)]
        >>> group = robot.main_group_name
        >>> robot.constraints_from_frame(frame, tolerance_position, tolerances_axes, group) # doctest: +NORMALIZE_WHITESPACE
        [PositionConstraint('ee_link', BoundingVolume(2, Sphere(Point(0.400, 0.300, 0.400), 0.001)), 1.0),
        OrientationConstraint('ee_link', [0.5, 0.5, 0.5, 0.5], [0.017453292519943295, 0.017453292519943295, 0.017453292519943295], 1.0)]
        """
        pc = self.position_constraint_from_frame(frame_WCF, tolerance_position, group, use_attached_tool_frame)
        oc = self.orientation_constraint_from_frame(frame_WCF, tolerances_axes, group, use_attached_tool_frame)
        return [pc, oc]

    def constraints_from_configuration(self, configuration, tolerances_above, tolerances_below, group=None):
        """Create joint constraints for all joints of the configuration.

        Parameters
        ----------
        configuration: :class:`Configuration`
            The target configuration.
        tolerances_above: :obj:`list` of :obj:`float`
            The tolerances above the targeted configuration's joint value on
            each of the joints, defining the upper bound in radians to be
            achieved. If only one value is passed, it will be used to create
            upper bounds for all joint constraints.
        tolerances_below: :obj:`list` of :obj:`float`
            The tolerances below the targeted configuration's joint value on
            each of the joints, defining the upper bound in radians to be
            achieved. If only one value is passed, it will be used to create
            lower bounds for all joint constraints.
        group: :obj:`str`, optional
            The planning group for which we specify the constraint. Defaults to
            the robot's main planning group.

        Returns
        -------
        :obj:`list` of :class:`JointConstraint`

        Raises
        ------
        :exc:`ValueError`
            If the passed configuration does not correspond to the group.
        :exc:`ValueError`
            If the passed list of tolerance values have a different length than
            the configuration.

        Notes
        -----
        Make sure that you are using the correct tolerance units if your robot
        has different joint types defined.

        Examples
        --------
        >>> configuration = Configuration.from_revolute_values([-0.042, 4.295, -4.110, -3.327, 4.755, 0.])
        >>> tolerances_above = [math.radians(1)] * 6
        >>> tolerances_below = [math.radians(1)] * 6
        >>> group = robot.main_group_name
        >>> robot.constraints_from_configuration(configuration, tolerances_above, tolerances_below, group)
        [JointConstraint('shoulder_pan_joint', -0.042, 0.017453292519943295, 0.017453292519943295, 1.0), \
        JointConstraint('shoulder_lift_joint', 4.295, 0.017453292519943295, 0.017453292519943295, 1.0), \
        JointConstraint('elbow_joint', -4.11, 0.017453292519943295, 0.017453292519943295, 1.0), \
        JointConstraint('wrist_1_joint', -3.327, 0.017453292519943295, 0.017453292519943295, 1.0), \
        JointConstraint('wrist_2_joint', 4.755, 0.017453292519943295, 0.017453292519943295, 1.0), \
        JointConstraint('wrist_3_joint', 0.0, 0.017453292519943295, 0.017453292519943295, 1.0)]
        """
        if not group:
            group = self.main_group_name

        joint_names = self.get_configurable_joint_names(group)
        if len(joint_names) != len(configuration.joint_values):
            raise ValueError(
                "The passed configuration has {} joint_values, the group {} needs however: {}".format(
                    len(configuration.joint_values), group, len(joint_names)
                )
            )
        if len(tolerances_above) == 1:
            tolerances_above = tolerances_above * len(joint_names)
        elif len(tolerances_above) != len(configuration.joint_values):
            raise ValueError(
                "The passed configuration has {} joint_values, the tolerances_above however: {}".format(
                    len(configuration.joint_values), len(tolerances_above)
                )
            )
        if len(tolerances_below) == 1:
            tolerances_below = tolerances_below * len(joint_names)
        elif len(tolerances_below) != len(configuration.joint_values):
            raise ValueError(
                "The passed configuration has {} joint_values, the tolerances_below however: {}".format(
                    len(configuration.joint_values), len(tolerances_below)
                )
            )

        constraints = []
        for name, value, tolerance_above, tolerance_below in zip(
            joint_names, configuration.joint_values, tolerances_above, tolerances_below
        ):
            constraints.append(JointConstraint(name, value, tolerance_above, tolerance_below))
        return constraints

    # ==========================================================================
    # services
    # ==========================================================================

    def inverse_kinematics(
        self,
        frame_WCF,
        start_configuration=None,
        group=None,
        return_full_configuration=False,
        use_attached_tool_frame=True,
        options=None,
    ):
        """Calculate the robot's inverse kinematic for a given frame.

        The inverse kinematic solvers are implemented as generators in order to fit both analytic
        and numerical solver approaches. However, this method abstracts that away and returns one
        configuration at a time to simplify its usage.

        To keep the usefulness of the generator, calls to this method will recall the last retrieved
        iterator and keep returning results yielded by it, one at a time. This is true only as long as
        the request is identical to the last one. If the arguments change, the last generator
        is discarded and the client IK implementation is invoked again.

        Parameters
        ----------
        frame_WCF : :class:`compas.geometry.Frame`
            The frame to calculate the inverse kinematic for.
        start_configuration : :class:`Configuration`, optional
            If passed, the inverse will be calculated such that the calculated
            joint positions differ the least from the start_configuration.
            Defaults to the zero configuration.
        group: :obj:`str`, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        return_full_configuration : :obj:`bool`, optional
            If ``True``, returns a full configuration with all joint values
            specified, including passive ones if available. Defaults to ``False``.
        use_attached_tool_frame : :obj:`bool`, optional
            If ``True`` and there is a tool attached to the planning group, it will use its TCF
            instead of the T0CF to calculate IK. Defaults to ``True``.
        options: :obj:`dict`, optional
            Dictionary containing the key-value pairs of additional options.
            The valid options are specific to the backend in use.
            Check the API reference of the IK backend implementation for more details.

        Raises
        ------
        :exc:`compas_fab.backends.BackendError`
            If no configuration can be found.

        Returns
        -------
        :class:`Configuration`
            An inverse kinematic solution represented as a configuration.

        Examples
        --------
        >>> frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
        >>> start_configuration = robot.zero_configuration()
        >>> group = robot.main_group_name
        >>> robot.inverse_kinematics(frame_WCF, start_configuration, group)                 # doctest: +SKIP
        Configuration((4.045, 5.130, -2.174, -6.098, -5.616, 6.283), (0, 0, 0, 0, 0, 0))    # doctest: +SKIP
        """
        # Pseudo-memoized sequential calls will re-use iterator if not exhaused
        request_id = "{}-{}-{}-{}-{}".format(
            str(frame_WCF), str(start_configuration), str(group), str(return_full_configuration), str(options)
        )

        if self._current_ik["request_id"] == request_id and self._current_ik["solutions"] is not None:
            solution = next(self._current_ik["solutions"], None)
            if solution is not None:
                return solution

        solutions = self.iter_inverse_kinematics(
            frame_WCF, start_configuration, group, return_full_configuration, use_attached_tool_frame, options
        )
        self._current_ik["request_id"] = request_id
        self._current_ik["solutions"] = solutions

        return next(solutions)

    def iter_inverse_kinematics(
        self,
        frame_WCF,
        start_configuration=None,
        group=None,
        return_full_configuration=False,
        use_attached_tool_frame=True,
        options=None,
    ):
        """Iterate over the inverse kinematic solutions of a robot.

        This method exposes the generator-based inverse kinematic solvers. Analytics solvers will return
        generators that include all possible solutions, hence exhausting the iterator indicates there are
        no more solutions to be found. Numerical solvers, on the other hand, will keep returning solutions
        until one fails to be found (usually caused by a timeout rather than infeasibility) or until the user
        code stops the iteration.

        Parameters
        ----------
        frame_WCF : :class:`compas.geometry.Frame`
            The frame to calculate the inverse kinematic for.
        start_configuration : :class:`Configuration`, optional
            If passed, the inverse will be calculated such that the calculated
            joint positions differ the least from the start_configuration.
            Defaults to the zero configuration.
        group: :obj:`str`, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        return_full_configuration : :obj:`bool`, optional
            If ``True``, returns a full configuration with all joint values
            specified, including passive ones if available. Defaults to ``False``.
        use_attached_tool_frame : :obj:`bool`, optional
            If ``True`` and there is a tool attached to the planning group, it will use its TCF
            instead of the T0CF to calculate IK. Defaults to ``True``.
        options: :obj:`dict`, optional
            Dictionary containing the key-value pairs of additional options.
            The valid options are specific to the backend in use.
            Check the API reference of the IK backend implementation for more details.

        Raises
        ------
        :exc:`compas_fab.backends.BackendError`
            If no configuration can be found.

        Yields
        -------
        :class:`Configuration`
            An inverse kinematic solution represented as a configuration.

        Examples
        --------
        >>> frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
        >>> start_configuration = robot.zero_configuration()
        >>> group = robot.main_group_name
        >>> next(robot.iter_inverse_kinematics(frame_WCF, start_configuration, group))      # doctest: +SKIP
        Configuration((4.045, 5.130, -2.174, -6.098, -5.616, 6.283), (0, 0, 0, 0, 0, 0))    # doctest: +SKIP
        """
        options = options or {}
        attached_collision_meshes = options.get("attached_collision_meshes") or []

        self.ensure_client()
        group = group or self.main_group_name if self.semantics else None

        start_configuration, start_configuration_scaled = self._check_full_configuration_and_scale(start_configuration)

        attached_tool = self.attached_tools.get(group)
        if use_attached_tool_frame and attached_tool:
            frame_WCF = self.from_tcf_to_t0cf([frame_WCF], group)[0]

        frame_WCF_scaled = frame_WCF.copy()
        frame_WCF_scaled.point /= self.scale_factor  # must be in meters

        for tool in self.attached_tools.values():
            if tool:
                attached_collision_meshes.extend(tool.attached_collision_meshes)

        options["attached_collision_meshes"] = attached_collision_meshes

        solutions = self.client.inverse_kinematics(self, frame_WCF_scaled, start_configuration_scaled, group, options)

        # The returned joint names might be more than the requested ones if there are passive joints present
        for joint_positions, joint_names in solutions:
            if joint_positions:
                yield self._build_configuration(joint_positions, joint_names, group, return_full_configuration)
            else:
                yield None  # to accomodate analytic ik with keeping the order of solutions

    def _build_configuration(self, joint_positions, joint_names, group, return_full_configuration):
        if return_full_configuration:
            # build configuration including passive joints, but no sorting
            joint_types = self.get_joint_types_by_names(joint_names)
            configuration = Configuration(joint_positions, joint_types, joint_names)
        else:
            # sort values for group configuration
            joint_state = dict(zip(joint_names, joint_positions))
            group_joint_names = self.get_configurable_joint_names(group)
            values = [joint_state[name] for name in group_joint_names]
            configuration = Configuration(values, self.get_configurable_joint_types(group), group_joint_names)

        return configuration.scaled(self.scale_factor)

    def inverse_kinematics_deprecated(
        self,
        frame_WCF,
        start_configuration=None,
        group=None,
        avoid_collisions=True,
        constraints=None,
        attempts=8,
        attached_collision_meshes=None,
        return_full_configuration=False,
    ):
        return self.inverse_kinematics(
            frame_WCF,
            start_configuration,
            group,
            return_full_configuration,
            options=dict(
                avoid_collisions=avoid_collisions,
                constraints=constraints,
                attempts=attempts,
                attached_collision_meshes=attached_collision_meshes,
            ),
        )

    def forward_kinematics(self, configuration, group=None, use_attached_tool_frame=True, options=None):
        """Calculate the robot's forward kinematic.

        Parameters
        ----------
        configuration: :class:`compas_fab.robots.Configuration`
            The full configuration to calculate the forward kinematic for. If no
            full configuration is passed, the zero-joint state for the other
            configurable joints is assumed.
        group: obj:`str`, optional
            The planning group used for the calculation. Defaults to the robot's
            main planning group.
        use_attached_tool_frame : :obj:`bool`, optional
            If ``True`` and there is a tool attached to the planning group, FK will return
            the TCF of the attached tool instead of the T0CF. Defaults to ``True``.
        options: obj:`dict`, optional
            Dictionary containing the following key-value pairs:

            - ``"solver"``: (:obj:`str`, optional) If ``None`` calculates FK
              with the client if it exists or with the robot model.
              If ``'model'`` use the robot model to calculate FK.
              Other values depend on specific backend implementation, some backends might
              allow selecting different FK solvers dynamically.
            - ``"link"``: (:obj:`str`, optional) The name of the link to
              calculate the forward kinematics for. Defaults to the group's end
              effector link.
              Backwards compatibility note: if there's no ``link`` option, the
              planner will try also ``ee_link`` as fallback before defaulting
              to the end effector's link.

            There are additional options that are specific to the backend in use.
            Check the API reference of the FK backend implementation for more details.

        Returns
        -------
        :class:`compas.geometry.Frame`
            The frame in the world's coordinate system (WCF).

        Raises
        ------
        :exc:`ValueError`
            If `link_name` doesn't match any of the :class:`Robot` instance's links.
        :exc:`NotImplementedError`
            If forward kinematic method for given `backend` is not implemented.

        Examples
        --------
        >>> configuration = Configuration.from_revolute_values([-2.238, -1.153, -2.174, 0.185, 0.667, 0.000])
        >>> group = robot.main_group_name
        >>> frame_WCF_c = robot.forward_kinematics(configuration, group)
        >>> options = {'solver': 'model'}
        >>> frame_WCF_m = robot.forward_kinematics(configuration, group, options)
        >>> frame_WCF_c == frame_WCF_m
        True
        """
        options = options or {}
        solver = options.get("solver")

        group = group or self.main_group_name if self.semantics else None

        full_configuration = self.merge_group_with_full_configuration(configuration, self.zero_configuration(), group)
        full_configuration, full_configuration_scaled = self._check_full_configuration_and_scale(full_configuration)

        # If there's no client, we default to `model` solver if there is no other assigned.
        if not self.client:
            solver = solver or "model"

        # Solve with the model
        if solver == "model":
            link = options.get("link", options.get("ee_link"))
            link = link or self.get_end_effector_link_name(group)
            if link not in self.get_link_names(group):
                raise ValueError("Link name {} does not exist in planning group".format(link))

            frame_WCF = self.model.forward_kinematics(full_configuration, link)

        # Otherwise, pass everything down to the client
        else:
            self.ensure_client()
            frame_WCF = self.client.forward_kinematics(self, full_configuration_scaled, group, options)

        # Scale and return
        frame_WCF.point *= self.scale_factor

        attached_tool = self.attached_tools.get(group)
        if use_attached_tool_frame and attached_tool:
            frame_WCF = self.from_t0cf_to_tcf([frame_WCF], group)[0]

        return frame_WCF

    def forward_kinematics_deprecated(self, configuration, group=None, backend=None, ee_link=None):
        return self.forward_kinematics(configuration, group, options=dict(solver=backend, link=ee_link))

    def plan_cartesian_motion(
        self, frames_WCF, start_configuration=None, group=None, use_attached_tool_frame=True, options=None
    ):
        """Calculate a cartesian motion path (linear in tool space).

        Parameters
        ----------
        frames_WCF : :obj:`list` of :class:`compas.geometry.Frame`
            The frames through which the path is defined.
        start_configuration : :class:`Configuration`, optional
            The robot's full configuration, i.e. values for all configurable
            joints of the entire robot, at the starting position. Defaults to
            the all-zero configuration.
        group : :obj:`str`, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        use_attached_tool_frame : :obj:`bool`, optional
            If ``True`` and there is a tool attached to the planning group, it will use its TCF
            instead of the T0CF to calculate cartesian paths. Defaults to ``True``.
        options : :obj:`dict`, optional
            Dictionary containing the following key-value pairs:

            - max_step :: :obj:`float`, optional
                The approximate distance between the
                calculated points. (Defined in the robot's units.) Defaults to ``0.01``.
            - path_constraints :: :obj:`list` of :class:`compas_fab.robots.Constraint`, optional
                Optional constraints that can be imposed along the solution path.
                Note that path calculation won't work if the start_configuration
                violates these constraints. Defaults to ``None``.
            - attached_collision_meshes :: :obj:`list` of :class:`compas_fab.robots.AttachedCollisionMesh`
                Defaults to ``None``.

            There are additional options that are specific to the backend in use.
            Check the API reference of the cartesian motion planner backend implementation
            for more details.

        Returns
        -------
        :class:`JointTrajectory`
            The calculated trajectory.

        Examples
        --------

        >>> ros = RosClient()
        >>> ros.run()
        >>> robot = ros.load_robot()
        >>> frames = [Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0]),\
                      Frame([0.5, 0.1, 0.6], [1, 0, 0], [0, 1, 0])]
        >>> start_configuration = Configuration.from_revolute_values([-0.042, 0.033, -2.174, 5.282, -1.528, 0.000])
        >>> group = robot.main_group_name
        >>> options = {'max_step': 0.01,\
                       'jump_threshold': 1.57,\
                       'avoid_collisions': True}
        >>> trajectory = robot.plan_cartesian_motion(frames,\
                                                     start_configuration,\
                                                     group=group,\
                                                     options=options)
        >>> len(trajectory.points) > 1
        True
        >>> ros.close()
        """
        options = options or {}
        max_step = options.get("max_step")
        path_constraints = options.get("path_constraints")
        attached_collision_meshes = options.get("attached_collision_meshes") or []

        self.ensure_client()
        if not group:
            group = self.main_group_name  # ensure semantics

        # NOTE: start_configuration has to be a full robot configuration, such
        # that all configurable joints of the whole robot are defined for planning.
        start_configuration, start_configuration_scaled = self._check_full_configuration_and_scale(start_configuration)

        attached_tool = self.attached_tools.get(group)
        if use_attached_tool_frame and attached_tool:
            frames_WCF = self.from_tcf_to_t0cf(frames_WCF, group)

        frames_WCF_scaled = []
        for frame in frames_WCF:
            frames_WCF_scaled.append(Frame(frame.point * 1.0 / self.scale_factor, frame.xaxis, frame.yaxis))

        if path_constraints:
            path_constraints_WCF_scaled = []
            for c in path_constraints:
                cp = c.copy()
                if c.type == Constraint.JOINT:
                    joint = self.get_joint_by_name(c.joint_name)
                    if joint.is_scalable():
                        cp.scale(1.0 / self.scale_factor)
                else:
                    cp.scale(1.0 / self.scale_factor)
                path_constraints_WCF_scaled.append(cp)
        else:
            path_constraints_WCF_scaled = None

        for _, tool in self.attached_tools.items():
            if tool:
                attached_collision_meshes.extend(tool.attached_collision_meshes)

        options["attached_collision_meshes"] = attached_collision_meshes
        options["path_constraints"] = path_constraints
        if max_step:
            options["max_step"] = max_step / self.scale_factor

        trajectory = self.client.plan_cartesian_motion(
            robot=self,
            frames_WCF=frames_WCF_scaled,
            start_configuration=start_configuration_scaled,
            group=group,
            options=options,
        )

        # Scale everything back to robot's scale
        for pt in trajectory.points:
            pt.scale(self.scale_factor)

        trajectory.start_configuration.scale(self.scale_factor)

        return trajectory

    def plan_cartesian_motion_deprecated(
        self,
        frames_WCF,
        start_configuration=None,
        max_step=0.01,
        jump_threshold=1.57,
        avoid_collisions=True,
        group=None,
        path_constraints=None,
        attached_collision_meshes=None,
    ):
        return self.plan_cartesian_motion(
            frames_WCF,
            start_configuration=start_configuration,
            group=group,
            options=dict(
                max_step=max_step,
                jump_threshold=jump_threshold,
                avoid_collisions=avoid_collisions,
                path_constraints=path_constraints,
                attached_collision_meshes=attached_collision_meshes,
            ),
        )

    def plan_motion(self, goal_constraints, start_configuration=None, group=None, options=None):
        """Calculate a motion path.

        Parameters
        ----------
        goal_constraints : list of :class:`Constraint`
            The goal to be achieved, defined in a set of constraints.
            Constraints can be very specific, for example defining value domains
            for each joint, such that the goal configuration is included,
            or defining a volume in space, to which a specific robot link (e.g.
            the end-effector) is required to move to.
        start_configuration : :class:`.Configuration`, optional
            The robot's full configuration, i.e. values for all configurable
            joints of the entire robot, at the starting position. Defaults to
            the all-zero configuration.
        group : :obj:`str`, optional
            The name of the group to plan for. Defaults to the robot's main
            planning group.
        options : :obj:`dict`, optional
            Dictionary containing the following key-value pairs:

            - path_constraints :: :obj:`list` of :class:`Constraint`, optional
                Optional constraints that can be imposed along the solution path.
                Note that path calculation won't work if the start_configuration
                violates these constraints. Defaults to ``None``.
            - attached_collision_meshes :: :obj:`list` of :class:`compas_fab.robots.AttachedCollisionMesh`
                Defaults to ``None``.

            There are additional options that are specific to the backend in use.
            Check the API reference of the motion planner backend implementation
            for more details.

        Returns
        -------
        :class:`JointTrajectory`
            The calculated trajectory.

        Examples
        --------

        Using position and orientation constraints:

        >>> ros = RosClient()
        >>> ros.run()
        >>> robot = ros.load_robot()
        >>> frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
        >>> tolerance_position = 0.001
        >>> tolerances_axes = [math.radians(1)] * 3
        >>> start_configuration = Configuration.from_revolute_values([-0.042, 4.295, 0, -3.327, 4.755, 0.])
        >>> group = robot.main_group_name
        >>> goal_constraints = robot.constraints_from_frame(frame, tolerance_position, tolerances_axes, group)
        >>> trajectory = robot.plan_motion(goal_constraints, start_configuration, group, {'planner_id': 'RRTConnect'})
        >>> trajectory.fraction
        1.0
        >>> len(trajectory.points) > 1
        True
        >>> ros.close()

        Using joint constraints (to the UP configuration):

        >>> ros = RosClient()
        >>> ros.run()
        >>> robot = ros.load_robot()
        >>> configuration = Configuration.from_revolute_values([0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0])
        >>> tolerances_above = [math.radians(5)] * len(configuration.joint_values)
        >>> tolerances_below = [math.radians(5)] * len(configuration.joint_values)
        >>> group = robot.main_group_name
        >>> goal_constraints = robot.constraints_from_configuration(configuration, tolerances_above, tolerances_below, group)
        >>> trajectory = robot.plan_motion(goal_constraints, start_configuration, group, {'planner_id': 'RRTConnect'})
        >>> trajectory.fraction
        1.0
        >>> len(trajectory.points) > 1
        True
        >>> ros.close()
        """
        options = options or {}
        path_constraints = options.get("path_constraints")
        attached_collision_meshes = options.get("attached_collision_meshes") or []

        # TODO: for the motion plan request a list of possible goal constraints
        # can be passed, from which the planner will try to find a path that
        # satisfies at least one of the specified goal constraints. For now only
        # one set of goal constraints is supported.

        # TODO: add workspace_parameters

        self.ensure_client()
        if not group:
            group = self.main_group_name

        # NOTE: start_configuration has to be a full robot configuration, such
        # that all configurable joints of the whole robot are defined for planning.
        start_configuration, start_configuration_scaled = self._check_full_configuration_and_scale(start_configuration)

        goal_constraints_WCF_scaled = []
        for c in goal_constraints:
            cp = c.copy()
            if c.type == Constraint.JOINT:
                joint = self.get_joint_by_name(c.joint_name)
                if joint.is_scalable():
                    cp.scale(1.0 / self.scale_factor)
            else:
                cp.scale(1.0 / self.scale_factor)
            goal_constraints_WCF_scaled.append(cp)

        # Transform path constraints to RCF and scale
        if path_constraints:
            path_constraints_WCF_scaled = []
            for c in path_constraints:
                cp = c.copy()
                if c.type == Constraint.JOINT:
                    joint = self.get_joint_by_name(c.joint_name)
                    if joint.is_scalable():
                        cp.scale(1.0 / self.scale_factor)
                else:
                    cp.scale(1.0 / self.scale_factor)
                path_constraints_WCF_scaled.append(cp)
        else:
            path_constraints_WCF_scaled = None

        for tool in self.attached_tools.values():
            if tool:
                attached_collision_meshes.extend(tool.attached_collision_meshes)

        options["attached_collision_meshes"] = attached_collision_meshes
        options["path_constraints"] = path_constraints_WCF_scaled

        trajectory = self.client.plan_motion(
            robot=self,
            goal_constraints=goal_constraints_WCF_scaled,
            start_configuration=start_configuration_scaled,
            group=group,
            options=options,
        )

        # Scale everything back to robot's scale
        for pt in trajectory.points:
            pt.scale(self.scale_factor)

        trajectory.start_configuration.scale(self.scale_factor)

        return trajectory

    def plan_motion_deprecated(
        self,
        goal_constraints,
        start_configuration=None,
        group=None,
        path_constraints=None,
        planner_id="RRTConnect",
        num_planning_attempts=1,
        allowed_planning_time=2.0,
        max_velocity_scaling_factor=1.0,
        max_acceleration_scaling_factor=1.0,
        attached_collision_meshes=None,
    ):
        return self.plan_motion(
            goal_constraints,
            start_configuration,
            group,
            options=dict(
                path_constraints=path_constraints,
                planner_id=planner_id,
                num_planning_attempts=num_planning_attempts,
                allowed_planning_time=allowed_planning_time,
                max_velocity_scaling_factor=max_velocity_scaling_factor,
                max_acceleration_scaling_factor=max_acceleration_scaling_factor,
                attached_collision_meshes=attached_collision_meshes,
            ),
        )

    def transformed_frames(self, configuration, group=None):
        """Get the robot's transformed frames.

        Parameters
        ----------
        configuration : :class:`Configuration`
            Configuration to compute transformed frames for.
        group : :obj:`str`, optional
            The planning group used for the calculation. Defaults to the robot's
            main planning group.

        Returns
        -------
        :obj:`list` of :class:`compas.geometry.Frame`
            Transformed frames.
        """
        if not len(configuration.joint_names):
            configuration.joint_names = self.get_configurable_joint_names(group)
        return self.model.transformed_frames(configuration)

    def transformed_axes(self, configuration, group=None):
        """Get the robot's transformed axes.

        Parameters
        ----------
        configuration : :class:`Configuration`
            Configuration to compute transformed axes for.
        group : :obj:`str`, optional
            The planning group used for the calculation. Defaults to the robot's
            main planning group.

        Returns
        -------
        :obj:`list` of :class:`compas.geometry.Vector`
            Transformed axes.
        """
        if not len(configuration.joint_names):
            configuration.joint_names = self.get_configurable_joint_names(group)
        return self.model.transformed_axes(configuration)

    # ==========================================================================
    # drawing
    # ==========================================================================

    def update(self, configuration, group=None, visual=True, collision=True):
        """Update the robot's geometry.

        Parameters
        ----------
        configuration : :class:`Configuration`
            Instance of the configuration (joint state) to move to.
        group : :obj:`str`, optional
            The name of the group to plan for. Defaults to the robot's main
            planning group.
        visual : :obj:`bool`, optional
            ``True`` if the visual geometry should be also updated, otherwise ``False``.
            Defaults to ``True``.
        collision : :obj:`bool`, optional
            ``True`` if the collision geometry should be also updated, otherwise ``False``.
            Defaults to ``True``.
        """
        group = group or self.main_group_name if self.semantics else None

        if not len(configuration.joint_names):
            configuration.joint_names = self.get_configurable_joint_names(group)

        self.artist.update(configuration, visual, collision)

    def draw_visual(self):
        """Draw the robot's visual geometry using the defined :attr:`Robot.artist`."""
        return self.artist.draw_visual()

    def draw_collision(self):
        """Draw the robot's collision geometry using the defined :attr:`Robot.artist`."""
        return self.artist.draw_collision()

    def draw(self):
        """Alias of :meth:`draw_visual`."""
        return self.draw_visual()

    # TODO: add artist.draw_attached_tool
    # def draw_attached_tool(self):
    #     """Draw the attached tool using the defined :attr:`Robot.artist`."""
    #     if self.artist and self.attached_tool:
    #         return self.artist.draw_attached_tool()

    def scale(self, factor):
        """Scale the robot geometry by a factor (absolute).

        Parameters
        ----------
        factor : :obj:`float`
            The factor to scale the robot with.

        Returns
        -------
        ``None``
        """
        self.model.scale(factor)
        if self.artist:
            self.artist.scale(factor)
        else:
            self._scale_factor = factor

    @property
    def scale_factor(self):
        """:obj:`float`: Robot's scale factor."""
        if self.artist:
            return self.artist.scale_factor
        else:
            return self._scale_factor

    def info(self):
        """Print information about the robot."""
        print("The robot's name is '{}'.".format(self.name))
        if self.semantics:
            print("The planning groups are:", self.group_names)
            print("The main planning group is '{}'.".format(self.main_group_name))
            configurable_joints = self.get_configurable_joints(self.main_group_name)
        else:
            configurable_joints = self.get_configurable_joints()
        print("The end-effector's name is '{}'.".format(self.get_end_effector_link_name()))
        if self.attached_tools:
            for tool in self.attached_tools.values():
                print("The robot has a tool at the {} link attached.".format(tool.link_name))
        else:
            print("The robot has NO tool attached.")
        print("The base link's name is '{}'".format(self.get_base_link_name()))
        print("The base_frame is:", self.get_base_frame())
        print("The robot's joints are:")
        for joint in configurable_joints:
            info = "\t* '{}' is of type '{}'".format(joint.name, list(Joint.SUPPORTED_TYPES)[joint.type])
            if joint.limit:
                info += " and has limits [{:.3f}, {:.3f}]".format(joint.limit.upper, joint.limit.lower)
            print(info)
        print("The robot's links are:")
        print([link.name for link in self.model.links])
