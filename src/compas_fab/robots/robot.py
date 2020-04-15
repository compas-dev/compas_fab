from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import logging
import random

from compas.geometry import Frame
from compas.geometry import Sphere
from compas.geometry import Transformation
from compas.robots import Joint
from compas.robots import RobotModel

from compas_fab.robots.configuration import Configuration
from compas_fab.robots.constraints import Constraint
from compas_fab.robots.constraints import JointConstraint
from compas_fab.robots.constraints import OrientationConstraint
from compas_fab.robots.constraints import PositionConstraint

from compas_fab.robots.planning_scene import AttachedCollisionMesh

LOGGER = logging.getLogger('compas_fab.robots.robot')

__all__ = [
    'Robot',
]


class Robot(object):
    """Represents a robot.

    This class binds together several building blocks, such as the robot's
    descriptive model, its semantic information and an instance of a backend
    client into a cohesive programmable interface. This representation builds
    upon the model described in the class :class:`compas.robots.RobotModel` of
    the **COMPAS** framework.

    Attributes
    ----------
    model : :class:`compas.robots.RobotModel`
        The robot model, usually created from an URDF structure.
    artist : :class:`compas_fab.artists.BaseRobotArtist`, optional
        Instance of the artist used to visualize the robot. Defaults to ``None``.
    semantics : :class:`RobotSemantics`, optional
        The semantic model of the robot. Defaults to ``None``.
    client : optional
        The backend client to use for communication,
        e.g. :class:`compas_fab.backends.RosClient`. Defaults to ``None``.
    """

    def __init__(self, model, artist=None, semantics=None, client=None):
        self._scale_factor = 1.
        self.model = model
        self.attached_tool = None
        self.artist = artist  # setter and getter (because of scale)
        self.semantics = semantics
        self.client = client  # setter and getter ?

    @property
    def artist(self):
        """:class:`compas_fab.artists.BaseRobotArtist`: Artist used to visualize robot."""
        return self._artist

    @artist.setter
    def artist(self, artist):
        self._artist = artist
        if len(self.model.joints) > 0 and len(self.model.links) > 0:
            self.scale(self._scale_factor)
            if self.attached_tool:
                self.artist.attach_tool(self.attached_tool)

    @classmethod
    def basic(cls, name, joints=None, links=None, materials=None, **kwargs):
        """Create the most basic instance of a robot, based only on name.

        Parameters
        ----------
        name : :class:`str`
            Name of the robot
        joints : :class:`list` of :class:`compas.robots.Joint`, optional
            Robot's joints.
        links : :class:`list` of :class:`compas.robots.Link`, optional
            Robot's links
        materials : :class:`list` of :class:`compas.robots.Material`, optional
            Material description of the robot.
        kwargs : :class:`dict`
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
        model = RobotModel(name, joints=joints or [], links=links or [],
                           materials=materials or [], **kwargs)
        return cls(model, None)

    @property
    def name(self):
        """:class:`str`: Name of the robot, as defined by its model.

        Examples
        --------
        >>> robot.name
        'ur5'
        """
        return self.model.name

    @property
    def group_names(self):
        """:class:`list` of :class:`str`: All planning groups of the robot.

        Examples
        --------
        >>> robot.group_names
        ['manipulator', 'endeffector']

        """
        self.ensure_semantics()
        return self.semantics.group_names

    @property
    def main_group_name(self):
        """:class:`str`: Robot's main planning group."""
        self.ensure_semantics()
        return self.semantics.main_group_name

    @property
    def root_name(self):
        """:class:`str`: Robot's root name."""
        return self.model.root.name

    def get_end_effector_link_name(self, group=None):
        """Get the name of the robot's end effector link.

        Parameters
        ----------
        group : :class:`str`, optional
            The name of the group. Defaults to the main planning group.

        Returns
        -------
        :class:`str`

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
        group : :class:`str`, optional
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
        group : :class:`str`, optional
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
        full_joint_state = dict(zip(full_configuration.joint_names, full_configuration.values))
        return self.model.forward_kinematics(full_joint_state, link_name=self.get_end_effector_link_name(group))

    def get_base_link_name(self, group=None):
        """Get the name of the robot's base link.

        Parameters
        ----------
        group : :class:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`str`

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
        group : :class:`str`, optional
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
        group : :class:`str`, optional
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
        full_joint_state = dict(zip(full_configuration.joint_names, full_configuration.values))
        return self.model.forward_kinematics(full_joint_state, link_name=self.get_base_link_name(group))

    def get_link_names(self, group=None):
        """Get the names of the links in the kinematic chain.

        Parameters
        ----------
        group : :class:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`list` of :class:`str`

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

    def get_configurable_joints(self, group=None):
        """Get the robot's configurable joints.

        Parameters
        ----------
        group : :class:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`list` of :class:`compas.robots.Joint`

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
        name : :class:`list` of :class:`str`
            The names of the joints.

        Returns
        -------
        :class:`list` of :attr:`compas.robots.Joint.SUPPORTED_TYPES`
            List of joint types.
        """
        return [self.get_joint_by_name(n).type for n in names]

    def get_joint_by_name(self, name):
        """Get the joint in the robot model matching the given name.

        Parameters
        ----------
        name : :class:`str`
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
        group : :class:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`list` of :class:`str`

        Note
        ----
        If :attr:`semantics` is set and no group is passed, it returns all
        configurable joints of all groups.

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
        group : :class:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`list` of :data:`compas.robots.Joint.SUPPORTED_TYPES`

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

    # ==========================================================================
    # configurations
    # ==========================================================================

    def zero_configuration(self, group=None):
        """Get the all-zero joint configuration.

        Examples
        --------
        >>> robot.zero_configuration('manipulator')
        Configuration((0.000, 0.000, 0.000, 0.000, 0.000, 0.000), (0, 0, 0, 0, 0, 0), \
            ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'))
        """
        joint_names = self.get_configurable_joint_names(group)
        joint_types = self.get_joint_types_by_names(joint_names)
        positions = [0.] * len(joint_types)
        return Configuration(positions, joint_types, joint_names)

    def random_configuration(self, group=None):
        """Get a random configuration.

        Parameters
        ----------
        group : :class:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`list` of :class:`float`

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
        group : :class:`str`
            The name of the planning group.
        full_configuration : :class:`Configuration`
            The configuration for all configurable joints of the robot.

        Returns
        -------
        :class:`Configuration`
            The configuration of the group.
        """
        full_configuration = self._check_full_configuration_and_scale(full_configuration)[0]  # adds joint_names to full_configuration and makes copy
        full_joint_state = dict(zip(full_configuration.joint_names, full_configuration.values))
        group_joint_names = self.get_configurable_joint_names(group)
        values = [full_joint_state[name] for name in group_joint_names]
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
        group : :class:`str`
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
        if not len(group_configuration.joint_names):
            group_configuration.joint_names = self.get_configurable_joint_names(group)

        full_configuration = self._check_full_configuration_and_scale(full_configuration)[0]  # adds joint_names to full_configuration and makes copy

        full_joint_state = dict(zip(full_configuration.joint_names, full_configuration.values))
        group_joint_state = dict(zip(group_configuration.joint_names, group_configuration.values))

        # overwrite full_joint_state with values of group_joint_state
        for name in group_joint_state:
            full_joint_state[name] = group_joint_state[name]

        full_configuration.values = [full_joint_state[name] for name in full_configuration.joint_names]
        return full_configuration

    def get_group_names_from_link_name(self, link_name):
        """Get the names of the groups `link_name` belongs to.

        Parameters
        ----------
        link_name : :class:`str`
            The name of a link

        Returns
        -------
        :class:`list` of :class:`str`
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
        joint_name : :class:`str`
            Name of joint.
        group : :class:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`float`
            Joint position for the given joint.

        Raises
        ------
        :exc:`ValueError`
            If the number of joints in the `configuration` parameter does not
            match the configurable joints of the given `group`.
        """
        names = self.get_configurable_joint_names(group)
        if len(names) != len(configuration.values):
            raise ValueError(
                "Please pass a configuration with %d values or specify group" % len(names))
        return configuration.values[names.index(joint_name)]

    def _check_full_configuration_and_scale(self, full_configuration=None):
        """Either creates a full configuration or checks if the passed full configuration is valid.

        Parameters
        ----------
        full_configuration : :class:`compas_fab.robots.Configuration`, optional
            The full configuration of the whole robot, including values for all configurable joints.

        Returns
        -------
        (:class:`compas_fab.robots.Configuration`, :class:`compas_fab.robots.Configuration`)
            The full configuration and the scaled full configuration
        """
        joint_names = self.get_configurable_joint_names()  # full configuration
        if not full_configuration:
            configuration = self.zero_configuration()  # with joint_names
        else:
            # full_configuration might have passive joints specified as well, we allow this.
            if len(joint_names) > len(full_configuration.values):
                raise ValueError("Please pass a configuration with {} values, for all configurable joints of the robot.".format(len(joint_names)))
            configuration = full_configuration.copy()
            if not len(configuration.joint_names):
                configuration.joint_names = joint_names
        return configuration, configuration.scaled(1. / self.scale_factor)
    # ==========================================================================
    # transformations, coordinate frames
    # ==========================================================================

    def transformation_RCF_WCF(self, group=None):
        """Get transformation from the robot's coordinate system (RCF) to the world coordinate system (WCF).

        Parameters
        ----------
        group : :class:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`compas.geometry.Transformation`

        """
        base_frame = self.get_base_frame(group)
        return Transformation.change_basis(base_frame, Frame.worldXY())

    def transformation_WCF_RCF(self, group=None):
        """Get the transformation from the world coordinate system (WCF) to the robot's coordinate system (RCF).

        Parameters
        ----------
        group : :class:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`compas.geometry.Transformation`

        """
        base_frame = self.get_base_frame(group)
        return Transformation.change_basis(Frame.worldXY(), base_frame)

    def set_RCF(self, robot_coordinate_frame, group=None):
        """Move the origin frame of the robot to the robot coordinate frame.

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
        group : :class:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`compas.geometry.Frame`
            Origin frame of the robot.
        """
        return self.get_base_frame(group)

    def to_local_coords(self, frame_WCF, group=None):
        """Represent a frame from the world coordinate system (WCF) in the robot's coordinate system (RCF).

        Parameters
        ----------
        frame_WCF : :class:`compas.geometry.Frame`
            A frame in the world coordinate frame.
        group : :class:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`compas.geometry.Frame`
            A frame in the robot's coordinate frame.

        Examples
        --------
        >>> frame_WCF = Frame([-0.363, 0.003, -0.147], [0.388, -0.351, -0.852], [0.276, 0.926, -0.256])
        >>> frame_RCF = robot.to_local_coords(frame_WCF)
        >>> frame_RCF
        Frame(Point(-0.363, 0.003, -0.147), Vector(0.388, -0.351, -0.852), Vector(0.276, 0.926, -0.256))
        """
        frame_RCF = frame_WCF.transformed(self.transformation_WCF_RCF(group))
        return frame_RCF

    def to_world_coords(self, frame_RCF, group=None):
        """Represent a frame from the robot's coordinate system (RCF) in the world coordinate system (WCF).

        Parameters
        ----------
        frame_RCF : :class:`compas.geometry.Frame`
            A frame in the robot's coordinate frame.
        group : :class:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`compas.geometry.Frame`
            A frame in the world coordinate frame.

        Examples
        --------
        >>> frame_RCF = Frame([-0.363, 0.003, -0.147], [0.388, -0.351, -0.852], [0.276, 0.926, -0.256])
        >>> frame_WCF = robot.to_world_coords(frame_RCF)
        >>> frame_WCF
        Frame(Point(-0.363, 0.003, -0.147), Vector(0.388, -0.351, -0.852), Vector(0.276, 0.926, -0.256))
        """
        frame_WCF = frame_RCF.transformed(self.transformation_RCF_WCF(group))
        return frame_WCF

    def from_attached_tool_to_tool0(self, frames_tcf):
        """Convert list of frames at the robot's tool tip (tcf frame) to frames at the robot's flange (tool0 frame) using the attached tool.

        Parameters
        ----------
        frames_tcf : :class:`list` of :class:`compas.geometry.Frame`
            Frames (in WCF) at the robot's tool tip (tcf).

        Returns
        -------
        :class:`list` of :class:`compas.geometry.Frame`
            Frames (in WCF) at the robot's flange (tool0).

        Raises
        ------
        :exc:`Exception`
            If the robot has no attached tool defined.

        Examples
        --------
        >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
        >>> frame = Frame([0.14, 0, 0], [0, 1, 0], [0, 0, 1])
        >>> robot.attach_tool(Tool(mesh, frame))
        >>> frames_tcf = [Frame((-0.309, -0.046, -0.266), (0.276, 0.926, -0.256), (0.879, -0.136, 0.456))]
        >>> robot.from_attached_tool_to_tool0(frames_tcf)
        [Frame(Point(-0.363, 0.003, -0.147), Vector(0.388, -0.351, -0.852), Vector(0.276, 0.926, -0.256))]
        """
        if not self.attached_tool:
            raise Exception("Please attach a tool first.")
        Te = Transformation.from_frame_to_frame(self.attached_tool.frame, Frame.worldXY())
        return [Frame.from_transformation(Transformation.from_frame(f) * Te) for f in frames_tcf]

    def from_tool0_to_attached_tool(self, frames_t0cf):
        """Convert frames at the robot's flange (tool0 frame) to frames at the robot's tool tip (tcf frame) using the attached tool.

        Parameters
        ----------
        frames_t0cf : :class:`list` of :class:`compas.geometry.Frame`
            Frames (in WCF) at the robot's flange (tool0).

        Returns
        -------
        :class:`list` of :class:`compas.geometry.Frame`
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
        >>> robot.from_tool0_to_attached_tool(frames_t0cf)
        [Frame(Point(-0.309, -0.046, -0.266), Vector(0.276, 0.926, -0.256), Vector(0.879, -0.136, 0.456))]
        """
        if not self.attached_tool:
            raise Exception("Please attach a tool first.")
        Te = Transformation.from_frame_to_frame(Frame.worldXY(), self.attached_tool.frame)
        return [Frame.from_transformation(Transformation.from_frame(f) * Te) for f in frames_t0cf]

    def attach_tool(self, tool, group=None, touch_links=None):
        """Attach a tool to the robot independently of the model definition.

        Parameters
        ----------
        tool : :class:`Tool`
            The tool that should be attached to the robot's flange.
        group : :class:`str`, optional
            The planning group to attach this tool to. Defaults to the main planning group.
        touch_links : :class:`list` of :class:`str`
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
        ee_link_name = self.get_end_effector_link_name(group)
        touch_links = touch_links or [ee_link_name]
        tool.attached_collision_mesh = AttachedCollisionMesh(tool.collision_mesh, ee_link_name, touch_links)
        self.attached_tool = tool
        if self.artist:
            self.update(self.zero_configuration(), group=group, visual=True, collision=True)  # TODO: this is not so ideal! should be called from within artist
            self.artist.attach_tool(tool)

    def detach_tool(self):
        """Detaches the attached tool.

        See Also
        --------
        * :meth:`attach_tool`
        """
        self.attached_tool = None
        if self.artist:
            self.artist.detach_tool()

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
            raise Exception(
                'This method is only callable once a client is assigned')

    def ensure_semantics(self):
        """Check if semantics is set.

        Raises
        ------
        :exc:`Exception`
            If :attr:`semantics` is not set.
        """
        if not self.semantics:
            raise Exception(
                'This method is only callable once a semantic model is assigned')

    # ==========================================================================
    # constraints
    # ==========================================================================

    def orientation_constraint_from_frame(self, frame_WCF, tolerances_axes, group=None):
        r"""Create an orientation constraint from a frame on the group's end-effector link.

        Parameters
        ----------
        frame_WCF: :class:`compas.geometry.Frame`
            The frame from which we create the orientation constraint.
        tolerances_axes: :class:`list` of :class:`float`
            Error tolerances t\ :sub:`i` for each of the frame's axes in
            radians. If only one value is passed it will be uses for all 3 axes.
        group: :class:`str`, optional
            The planning group for which we specify the constraint. Defaults to
            the robot's main planning group.

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
        ee_link = self.get_end_effector_link_name(group)

        tolerances_axes = list(tolerances_axes)
        if len(tolerances_axes) == 1:
            tolerances_axes *= 3
        elif len(tolerances_axes) != 3:
            raise ValueError("Must give either one or 3 values")
        return OrientationConstraint(ee_link, frame_WCF.quaternion, tolerances_axes)

    def position_constraint_from_frame(self, frame_WCF, tolerance_position, group=None):
        """Create a position constraint from a frame on the group's end-effector link.

        Parameters
        ----------
        frame_WCF : :class:`compas.geometry.Frame`
            The frame from which we create position and orientation constraints.
        tolerance_position : :class:`float`
            The allowed tolerance to the frame's position (defined in the
            robot's units).
        group: :class:`str`, optional
            The planning group for which we specify the constraint. Defaults to
            the robot's main planning group.

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
        ee_link = self.get_end_effector_link_name(group)
        sphere = Sphere(frame_WCF.point, tolerance_position)
        return PositionConstraint.from_sphere(ee_link, sphere)

    def constraints_from_frame(self, frame_WCF, tolerance_position, tolerances_axes, group=None):
        r"""Create a position and an orientation constraint from a frame calculated for the group's end-effector link.

        Parameters
        ----------
        frame_WCF: :class:`compas.geometry.Frame`
            The frame from which we create position and orientation constraints.
        tolerance_position: :class:`float`
            The allowed tolerance to the frame's position (defined in the
            robot's units).
        tolerances_axes: :class:`list` of :class:`float`
            Error tolerances t\ :sub:`i` for each of the frame's axes in
            radians. If only one value is passed it will be uses for all 3 axes.
        group: :class:`str`, optional
            The planning group for which we specify the constraint. Defaults to
            the robot's main planning group.

        Returns
        -------
        :class:`list` of :class:`Constraint`

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
        >>> robot.constraints_from_frame(frame, tolerance_position, tolerances_axes, group)
        [PositionConstraint('ee_link', BoundingVolume(2, Sphere(Point(0.400, 0.300, 0.400), 0.001)), 1.0), OrientationConstraint('ee_link', [0.5, 0.5, 0.5, 0.5], [0.017453292519943295, 0.017453292519943295, 0.017453292519943295], 1.0)]
        """  # noqa: E501
        pc = self.position_constraint_from_frame(frame_WCF, tolerance_position, group)
        oc = self.orientation_constraint_from_frame(frame_WCF, tolerances_axes, group)
        return [pc, oc]

    def constraints_from_configuration(self, configuration, tolerances_above, tolerances_below, group=None):
        """Create joint constraints for all joints of the configuration.

        Parameters
        ----------
        configuration: :class:`Configuration`
            The target configuration.
        tolerances_above: :class:`list` of :class:`float`
            The tolerances above the targeted configuration's joint value on each
            of the joints, defining the upper bound in radian to be achieved.
            If only one value is passed, it will be used to create upper bounds
            for all joint constraints.
        tolerances_below: :class:`list` of :class:`float`
            The tolerances below the targeted configuration's joint value on each
            of the joints, defining the upper bound in radian to be achieved.
            If only one value is passed, it will be used to create lower bounds
            for all joint constraints.
        group: :class:`str`, optional
            The planning group for which we specify the constraint. Defaults to
            the robot's main planning group.

        Returns
        -------
        :class:`list` of :class:`JointConstraint`

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
        if len(joint_names) != len(configuration.values):
            raise ValueError("The passed configuration has %d values, the group %s needs however: %d" % (
                len(configuration.values), group, len(joint_names)))
        if len(tolerances_above) == 1:
            tolerances_above = tolerances_above * len(joint_names)
        elif len(tolerances_above) != len(configuration.values):
            raise ValueError("The passed configuration has %d values, the tolerances_above however: %d" % (
                len(configuration.values), len(tolerances_above)))
        if len(tolerances_below) == 1:
            tolerances_below = tolerances_below * len(joint_names)
        elif len(tolerances_below) != len(configuration.values):
            raise ValueError("The passed configuration has %d values, the tolerances_below however: %d" % (
                len(configuration.values), len(tolerances_below)))

        constraints = []
        for name, value, tolerance_above, tolerance_below in zip(joint_names, configuration.values, tolerances_above, tolerances_below):
            constraints.append(JointConstraint(name, value, tolerance_above, tolerance_below))
        return constraints

    # ==========================================================================
    # services
    # ==========================================================================

    def inverse_kinematics(self, frame_WCF, start_configuration=None,
                           group=None, avoid_collisions=True,
                           constraints=None, attempts=8,
                           attached_collision_meshes=None,
                           return_full_configuration=False):
        """Calculate the robot's inverse kinematic for a given frame.

        Parameters
        ----------
        frame : :class:`compas.geometry.Frame`
            The frame to calculate inverse kinematic for.
        start_configuration : :class:`Configuration`, optional
            If passed, the inverse will be calculated such that the calculated
            joint positions differ the least from the start_configuration.
            Defaults to the initial configuration.
        group : :class:`str`, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        avoid_collisions : :class:`bool`, optional
            Whether or not to avoid collisions. Defaults to ``True``.
        constraints : :class:`list` of :class:`Constraint`, optional
            A set of constraints that the request must obey. Defaults to ``None``.
        attempts : :class:`int`, optional
            The maximum number of inverse kinematic attempts. Defaults to 8.
        attached_collision_meshes : :class:`list` of :class:`AttachedCollisionMesh`, optional
            Defaults to ``None``.
        return_full_configuration : :class:`bool`
            If ``True``, returns a full configuration with all joint values
            specified, including passive ones if available.

        Returns
        -------
        :class:`Configuration`
            The planning group's configuration.

        Raises
        ------
        :exc:`compas_fab.backends.BackendError`
            If no configuration can be found.

        Examples
        --------
        >>> frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
        >>> start_configuration = robot.zero_configuration()
        >>> group = robot.main_group_name
        >>> robot.inverse_kinematics(frame_WCF, start_configuration, group)                 # doctest: +SKIP
        Configuration((4.045, 5.130, -2.174, -6.098, -5.616, 6.283), (0, 0, 0, 0, 0, 0))    # doctest: +SKIP
        """
        self.ensure_client()
        if not group:
            group = self.main_group_name  # ensure semantics

        start_configuration, start_configuration_scaled = self._check_full_configuration_and_scale(start_configuration)

        frame_WCF_scaled = frame_WCF.copy()
        frame_WCF_scaled.point /= self.scale_factor  # must be in meters

        if self.attached_tool:
            if attached_collision_meshes:
                attached_collision_meshes.append(self.attached_tool.attached_collision_mesh)
            else:
                attached_collision_meshes = [self.attached_tool.attached_collision_mesh]

        # The returned joint names might be more than the requested ones if there are passive joints present
        joint_positions, joint_names = self.client.inverse_kinematics(self,
                                                                      frame_WCF_scaled,
                                                                      group, start_configuration_scaled,
                                                                      avoid_collisions, constraints, attempts,
                                                                      attached_collision_meshes)
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

    def forward_kinematics(self, configuration, group=None, backend=None, link_name=None):
        """Calculate the robot's forward kinematic.

        Parameters
        ----------
        full_configuration : :class:`Configuration`
            The full configuration to calculate the forward kinematic for. If no
            full configuration is passed, the zero-joint state for the other
            configurable joints is assumed.
        group : :class:`str`, optional
            The planning group used for the calculation. Defaults to the robot's
            main planning group.
        backend : ``None`` or :class:`str`
            * If ``None``, calculates forward kinematic with the client if it exists or
              with the robot model.
            * If ``'model'``, use the robot model to calculate forward kinematic.

            Anything else is open for implementation, possibly ``'kdl'`` or
            ``'ikfast'``.
        link_name : :class:`str`, optional
            The name of the link to calculate the forward kinematics for.
            Defaults to the group's end effector link.

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
        >>> frame_WCF_m = robot.forward_kinematics(configuration, group, backend='model')
        >>> frame_WCF_c == frame_WCF_m
        True
        """
        if not group:
            group = self.main_group_name

        if link_name is None:
            link_name = self.get_end_effector_link_name(group)
        else:
            # check
            if link_name not in self.get_link_names(group):
                raise ValueError("Link name %s does not exist in planning group" % link_name)

        full_configuration = self.merge_group_with_full_configuration(configuration, self.zero_configuration(), group)
        full_configuration, full_configuration_scaled = self._check_full_configuration_and_scale(full_configuration)

        full_joint_state = dict(zip(full_configuration.joint_names, full_configuration.values))

        if not backend:
            if self.client:
                frame_WCF = self.client.forward_kinematics(self,
                                                           full_configuration_scaled,
                                                           group,
                                                           link_name)
                frame_WCF.point *= self.scale_factor
            else:
                frame_WCF = self.model.forward_kinematics(full_joint_state, link_name)
        elif backend == 'model':
            frame_WCF = self.model.forward_kinematics(full_joint_state, link_name)
        else:
            # pass to backend, kdl, ikfast,...
            raise NotImplementedError

        return frame_WCF

    def plan_cartesian_motion(self, frames_WCF, start_configuration=None,
                              max_step=0.01, jump_threshold=1.57,
                              avoid_collisions=True, group=None,
                              path_constraints=None,
                              attached_collision_meshes=None):
        """Calculate a cartesian motion path (linear path in tool space).

        Parameters
        ----------
        frames_WCF : :class:`list` of :class:`compas.geometry.Frame`
            The frames through which the path is defined.
        start_configuration : :class:`Configuration`, optional
            The robot's full configuration, i.e. values for all configurable
            joints of the entire robot, at the starting position. Defaults to
            the all-zero configuration.
        max_step : :class:`float`, optional
            The approximate distance between the calculated points. (Defined in
            the robot's units)
        jump_threshold : :class:`float`, optional
            The maximum allowed distance of joint positions between consecutive
            points. If the distance is found to be above this threshold, the
            path computation fails. It must be specified in relation to max_step.
            If this theshhold is 0, 'jumps' might occur, resulting in an invalid
            cartesian path. Defaults to pi / 2.
        avoid_collisions : :class:`bool`, optional
            Whether or not to avoid collisions. Defaults to ``True``.
        group : :class:`str`, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        path_constraints : :class:`list` of :class:`Constraint`, optional
            Optional constraints that can be imposed along the solution path.
            Note that path calculation won't work if the start_configuration
            violates these constraints. Defaults to ``None``.
        attached_collision_meshes : :class:`list` of :class:`AttachedCollisionMesh`, optional
            Defaults to ``None``.

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
            The calculated trajectory.

        Examples
        --------
        >>> frames = [Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0]),\
                      Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])]
        >>> start_configuration = Configuration.from_revolute_values([-0.042, 4.295, -4.110, -3.327, 4.755, 0.])
        >>> group = robot.main_group_name
        >>> trajectory = robot.plan_cartesian_motion(frames,\
                                                     start_configuration,\
                                                     max_step=0.01,\
                                                     jump_threshold=1.57,\
                                                     avoid_collisions=True,\
                                                     group=group)
        >>> type(trajectory)
        <class 'compas_fab.robots.trajectory.JointTrajectory'>
        """
        self.ensure_client()
        if not group:
            group = self.main_group_name  # ensure semantics

        # NOTE: start_configuration has to be a full robot configuration, such
        # that all configurable joints of the whole robot are defined for planning.
        start_configuration, start_configuration_scaled = self._check_full_configuration_and_scale(start_configuration)

        max_step_scaled = max_step / self.scale_factor

        frames_WCF_scaled = []
        for frame in frames_WCF:
            frames_WCF_scaled.append(Frame(frame.point * 1. / self.scale_factor, frame.xaxis, frame.yaxis))

        if path_constraints:
            path_constraints_WCF_scaled = []
            for c in path_constraints:
                cp = c.copy()
                if c.type == Constraint.JOINT:
                    joint = self.get_joint_by_name(c.joint_name)
                    if joint.is_scalable():
                        cp.scale(1. / self.scale_factor)
                else:
                    cp.scale(1. / self.scale_factor)
                path_constraints_WCF_scaled.append(cp)
        else:
            path_constraints_WCF_scaled = None

        if self.attached_tool:
            if attached_collision_meshes:
                attached_collision_meshes.append(self.attached_tool.attached_collision_mesh)
            else:
                attached_collision_meshes = [self.attached_tool.attached_collision_mesh]

        trajectory = self.client.plan_cartesian_motion(
            robot=self,
            frames=frames_WCF_scaled,
            start_configuration=start_configuration_scaled,
            group=group,
            max_step=max_step_scaled,
            jump_threshold=jump_threshold,
            avoid_collisions=avoid_collisions,
            path_constraints=path_constraints_WCF_scaled,
            attached_collision_meshes=attached_collision_meshes)

        # Scale everything back to robot's scale
        for pt in trajectory.points:
            pt.scale(self.scale_factor)

        trajectory.start_configuration.scale(self.scale_factor)

        return trajectory

    def plan_motion(self, goal_constraints, start_configuration=None,
                    group=None, path_constraints=None, planner_id='RRT',
                    num_planning_attempts=1, allowed_planning_time=2.,
                    max_velocity_scaling_factor=1.,
                    max_acceleration_scaling_factor=1.,
                    attached_collision_meshes=None):
        """Calculate a motion path.

        Parameters
        ----------
        goal_constraints : :class:`list` of :class:`Constraint`
            The goal to be achieved, defined in a set of constraints.
            Constraints can be very specific, for example defining value domains
            for each joint, such that the goal configuration is included,
            or defining a volume in space, to which a specific robot link (e.g.
            the end-effector) is required to move to.
        start_configuration : :class:`Configuration`, optional
            The robot's full configuration, i.e. values for all configurable
            joints of the entire robot, at the starting position. Defaults to
            the all-zero configuration.
        group : :class:`str`, optional
            The name of the group to plan for. Defaults to the robot's main
            planning group.
        path_constraints : :class:`list` of :class:`Constraint`, optional
            Optional constraints that can be imposed along the solution path.
            Note that path calculation won't work if the `start_configuration`
            violates these constraints. Defaults to ``None``.
        planner_id : :class:`str`
            The name of the algorithm used for path planning. Defaults to ``'RRT'``.
        num_planning_attempts : :class:`int`, optional
            Normally, if one motion plan is needed, one motion plan is computed.
            However, for algorithms that use randomization in their execution
            (like ``'RRT'``), it is likely that different planner executions will
            produce different solutions. Setting this parameter to a value above
            1 will run many additional motion plans, and will report the
            shortest solution as the final result. Defaults to 1.
        allowed_planning_time : :class:`float`
            The number of seconds allowed to perform the planning. Defaults to 2.
        max_velocity_scaling_factor : :class:`float`
            Defaults to 1.
        max_acceleration_scaling_factor : :class:`float`
            Defaults to 1.
        attached_collision_meshes : :class:`list` of :class:`AttachedCollisionMesh`
            Defaults to ``None``.

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
            The calculated trajectory.

        Examples
        --------
        >>> # Example with position and orientation constraints
        >>> frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
        >>> tolerance_position = 0.001
        >>> tolerances_axes = [math.radians(1)] * 3
        >>> start_configuration = Configuration.from_revolute_values([-0.042, 4.295, 0, -3.327, 4.755, 0.])
        >>> group = robot.main_group_name
        >>> goal_constraints = robot.constraints_from_frame(frame, tolerance_position, tolerances_axes, group)
        >>> robot.attached_tool = None
        >>> trajectory = robot.plan_motion(goal_constraints, start_configuration, group, planner_id='RRT')
        >>> trajectory.fraction
        1.0
        >>> # Example with joint constraints (to the UP configuration)
        >>> configuration = Configuration.from_revolute_values([0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0])
        >>> tolerances_above = [math.radians(5)] * len(configuration.values)
        >>> tolerances_below = [math.radians(5)] * len(configuration.values)
        >>> group = robot.main_group_name
        >>> goal_constraints = robot.constraints_from_configuration(configuration, tolerances_above, tolerances_below, group)
        >>> trajectory = robot.plan_motion(goal_constraints, start_configuration, group, planner_id='RRT')
        >>> trajectory.fraction
        1.0
        >>> type(trajectory)
        <class 'compas_fab.robots.trajectory.JointTrajectory'>
        """
        # TODO: for the motion plan request a list of possible goal constraints
        # can be passed, from which the planner will try to find a path that
        # satisfies at least one of the specified goal constraints. For now only
        # one set of goal constraints is supported.

        # TODO: add workspace_parameters

        self.ensure_client()
        if not group:
            group = self.main_group_name  # ensure semantics

        # NOTE: start_configuration has to be a full robot configuration, such
        # that all configurable joints of the whole robot are defined for planning.
        start_configuration, start_configuration_scaled = self._check_full_configuration_and_scale(start_configuration)

        goal_constraints_WCF_scaled = []
        for c in goal_constraints:
            cp = c.copy()
            if c.type == Constraint.JOINT:
                joint = self.get_joint_by_name(c.joint_name)
                if joint.is_scalable():
                    cp.scale(1. / self.scale_factor)
            else:
                cp.scale(1. / self.scale_factor)
            goal_constraints_WCF_scaled.append(cp)

        # Transform path constraints to RCF and scale
        if path_constraints:
            path_constraints_WCF_scaled = []
            for c in path_constraints:
                cp = c.copy()
                if c.type == Constraint.JOINT:
                    joint = self.get_joint_by_name(c.joint_name)
                    if joint.is_scalable():
                        cp.scale(1. / self.scale_factor)
                else:
                    cp.scale(1. / self.scale_factor)
                path_constraints_WCF_scaled.append(cp)
        else:
            path_constraints_WCF_scaled = None

        if self.attached_tool:
            if attached_collision_meshes:
                attached_collision_meshes.append(self.attached_tool.attached_collision_mesh)
            else:
                attached_collision_meshes = [self.attached_tool.attached_collision_mesh]

        trajectory = self.client.plan_motion(
            robot=self,
            goal_constraints=goal_constraints_WCF_scaled,
            start_configuration=start_configuration_scaled,
            group=group,
            path_constraints=path_constraints_WCF_scaled,
            trajectory_constraints=None,
            planner_id=planner_id,
            num_planning_attempts=num_planning_attempts,
            allowed_planning_time=allowed_planning_time,
            max_velocity_scaling_factor=max_velocity_scaling_factor,
            max_acceleration_scaling_factor=max_acceleration_scaling_factor,
            attached_collision_meshes=attached_collision_meshes,
            workspace_parameters=None)

        # Scale everything back to robot's scale
        for pt in trajectory.points:
            pt.scale(self.scale_factor)

        trajectory.start_configuration.scale(self.scale_factor)

        return trajectory

    def transformed_frames(self, configuration, group=None):
        """Get the robot's transformed frames.

        Parameters
        ----------
        configuration : :class:`Configuration`
            Configuration to compute transformed frames for.
        group : :class:`str`, optional
            The planning group used for the calculation. Defaults to the robot's
            main planning group.

        Returns
        -------
        :class:`list` of :class:`compas.geometry.Frame`
        """
        if not len(configuration.joint_names):
            configuration.joint_names = self.get_configurable_joint_names(group)

        joint_state = dict(zip(configuration.joint_names, configuration.values))

        return self.model.transformed_frames(joint_state)

    def transformed_axes(self, configuration, group=None):
        """Get the robot's transformed axes.

        Parameters
        ----------
        configuration : :class:`Configuration`
            Configuration to compute transformed axes for.
        group : :class:`str`, optional
            The planning group used for the calculation. Defaults to the robot's
            main planning group.

        Returns
        -------
        :class:`list` of :class:`compas.geometry.Vector`
        """
        if not len(configuration.joint_names):
            configuration.joint_names = self.get_configurable_joint_names(group)

        joint_state = dict(zip(configuration.joint_names, configuration.values))

        return self.model.transformed_axes(joint_state)

    # ==========================================================================
    # drawing
    # ==========================================================================

    def update(self, configuration, group=None, visual=True, collision=True):
        """Update the robot's geometry.

        Parameters
        ----------
        configuration : :class:`Configuration`
            Instance of the configuration (joint state) to move to.
        group : :class:`str`, optional
            The name of the group to plan for. Defaults to the robot's main
            planning group.
        visual : :class:`bool`, optional
            ``True`` if the visual geometry should be also updated, otherwise ``False``.
            Defaults to ``True``.
        collision : :class:`bool`, optional
            ``True`` if the collision geometry should be also updated, otherwise ``False``.
            Defaults to ``True``.
        """
        if not len(configuration.joint_names):
            configuration.joint_names = self.get_configurable_joint_names(group)
        self.artist.update(configuration, visual, collision)

    def draw_visual(self):
        """Draws the visual geometry of the robot in the defined :attr:`.artist`'s CAD environment."""
        return self.artist.draw_visual()

    def draw_collision(self):
        """Draws the collision geometry of the robot in the defined :attr:`.artist`'s CAD environment."""
        return self.artist.draw_collision()

    def draw(self):
        """Draws the visual geometry of the robot in the defined :attr:`.artist`'s CAD environment."""
        return self.draw_visual()

    def draw_attached_tool(self):
        """Draws the attached tool in the defined :attr:`artist`'s CAD environment."""
        if self.artist and self.attached_tool:
            return self.artist.draw_attached_tool()

    def scale(self, factor):
        """Scales the robot geometry by factor (absolute).

        Parameters
        ----------
        factor : :class:`float`
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
        """:class:`float`: Robot's scale factor."""
        if self.artist:
            return self.artist.scale_factor
        else:
            return self._scale_factor

    def info(self):
        """Print information about the robot."""
        print("The robot's name is '%s'." % self.name)
        if self.semantics:
            print("The planning groups are:", self.group_names)
            print("The main planning group is '%s'." % self.main_group_name)
            configurable_joints = self.get_configurable_joints(
                self.main_group_name)
        else:
            configurable_joints = self.get_configurable_joints()
        print("The end-effector's name is '%s'." %
              self.get_end_effector_link_name())
        if self.attached_tool:
            print("The robot has a tool at the %s link attached." % self.attached_tool.attached_collision_mesh.link_name)
        else:
            print("The robot has NO tool attached.")
        print("The base link's name is '%s'" % self.get_base_link_name())
        print("The base_frame is:", self.get_base_frame())
        print("The robot's joints are:")
        for joint in configurable_joints:
            info = "\t* '%s' is of type '%s'" % (
                joint.name, list(Joint.SUPPORTED_TYPES)[joint.type])
            if joint.limit:
                info += " and has limits [%.3f, %.3f]" % (
                    joint.limit.upper, joint.limit.lower)
            print(info)
        print("The robot's links are:")
        print([l.name for l in self.model.links])
