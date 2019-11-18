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
from compas.datastructures import mesh_transformed

from compas_fab.robots.configuration import Configuration
from compas_fab.robots.constraints import Constraint
from compas_fab.robots.constraints import JointConstraint
from compas_fab.robots.constraints import OrientationConstraint
from compas_fab.robots.constraints import PositionConstraint

from compas_fab.robots.planning_scene import CollisionMesh
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
    artist : :class:`BaseRobotArtist`, optional
        Instance of the artist used to visualize the robot. Defaults to ``None``.
    semantics : :class:`compas_fab.robots.RobotSemantics`, optional
        The semantic model of the robot. Defaults to ``None``.
    client : optional
        The backend client to use for communication,
        e.g. :class:`compas_fab.backends.RosClient`
    """

    def __init__(self, model, artist=None, semantics=None, client=None):
        self._scale_factor = 1.
        self.model = model
        self.artist = artist  # setter and getter (because of scale)
        self.semantics = semantics
        self.client = client  # setter and getter ?
        self.end_effector = None
        self.end_effector_frame = None

    @property
    def artist(self):
        """The artist which is used to visualize the robot."""
        return self._artist

    @artist.setter
    def artist(self, artist):
        self._artist = artist
        self.scale(self._scale_factor)

    @classmethod
    def basic(cls, name, joints=[], links=[], materials=[], **kwargs):
        """Convenience method to create the most basic instance of a robot, based only on a name.

        Parameters
        ----------
        name : str
            Name of the robot

        Returns
        -------
        :class:`Robot`
            Newly created instance of a robot.
        """
        model = RobotModel(name, joints=joints, links=links,
                           materials=materials, **kwargs)
        return cls(model, None)

    @property
    def name(self):
        """Name of the robot, as defined by its model

        Returns
        -------
        str
            Name of the robot.

        Examples
        --------
        >>> robot.name
        'ur5'
        """
        return self.model.name

    @property
    def group_names(self):
        """All planning groups of the robot.

        Examples
        --------
        >>> robot.group_names
        ['manipulator', 'endeffector']

        """
        self.ensure_semantics()
        return self.semantics.group_names

    @property
    def main_group_name(self):
        """The robot's main planning group."""
        self.ensure_semantics()
        return self.semantics.main_group_name

    @property
    def root_name(self):
        """The robot's root name."""
        return self.model.root.name

    def get_end_effector_link_name(self, group=None):
        """Returns the name of the end effector link.

        Parameters
        ----------
        group : str
            The name of the group. Defaults to `None`.

        Returns
        -------
        str

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
        """Returns the end effector link.

        Parameters
        ----------
        group : str
            The name of the group. Defaults to `None`.

        Returns
        -------
        :class: `compas.robots.Link`

        Examples
        --------
        >>> link = robot.get_end_effector_link()
        >>> link.name
        'ee_link'
        """
        name = self.get_end_effector_link_name(group)
        return self.model.get_link_by_name(name)

    def get_end_effector_frame(self, group=None):
        """Returns the end effector's frame.

        Parameters
        ----------
        group : str
            The name of the group. Defaults to `None`.

        Returns
        -------
        :class: `compas.geometry.Frame`
        """
        link = self.get_end_effector_link(group)
        return link.parent_joint.origin.copy()

    def get_base_link_name(self, group=None):
        """Returns the name of the base link.

        Parameters
        ----------
        group : str
            The name of the group. Defaults to `None`.

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
        """Returns the base link.

        Parameters
        ----------
        group : str
            The name of the group. Defaults to `None`.

        Returns
        -------
        :class: `compas.robots.Link`

        Examples
        --------
        >>> link = robot.get_base_link()
        >>> link.name
        'base_link'
        """
        name = self.get_base_link_name(group)
        return self.model.get_link_by_name(name)

    def get_base_frame(self, group=None):
        """Returns the frame of the base link, which is the robot's origin frame.

        Parameters
        ----------
        group : str
            The name of the group. Defaults to `None`.

        Returns
        -------
        :class: `compas.geometry.Frame`
        """
        # TODO: check this
        link = self.get_base_link(group)
        if link.parent_joint:
            base_frame = link.parent_joint.origin.copy()
        else:
            base_frame = Frame.worldXY()
        if not self.artist:
            base_frame.point *= self._scale_factor
        return base_frame

    def _get_current_base_frame(self, full_configuration, group):
        """Returns the group's current base frame, if the robot is in full_configuration.

        The base_frame of a planning group can change if a parent joint was
        transformed. This function performs a forward kinematic request with the
        full configuration to retrieve the (possibly) transformed base_frame of
        planning group. This function is only used in plan_motion since other
        services, such as ik or plan_cartesian_motion, do not use the
        transformed base_frame as the group's local coordinate system.

        Parameters
        ----------
        full_configuration : :class:`compas_fab.robots.Configuration`
            The (full) configuration from which the group's base frame is
            calculated.
        group : str
            The planning group for which we want to get the transformed base frame.

        Returns
        -------
        :class:`compas.geometry.Frame`

        Examples
        --------
        """

        base_link = self.get_base_link(group)
        # the group's original base_frame
        base_frame = self.get_base_frame(group)

        joint_names = self.get_configurable_joint_names()
        joint_positions = self._get_scaled_joint_positions_from_start_configuration(full_configuration)

        # ideally we would call this with the planning group that includes all
        # configurable joints, but we cannot be sure that this group exists.
        # That's why we have to do the workaround with the Transformation.

        joint_state = dict(zip(joint_names, joint_positions))

        if not base_link.parent_joint:
            base_frame_WCF = Frame.worldXY()
        else:
            base_frame_WCF = self.model.forward_kinematics(joint_state, link_name=base_link.name)
        base_frame_RCF = self.to_local_coords(base_frame_WCF, group)

        base_frame_RCF.point *= self.scale_factor
        T = Transformation.from_frame(base_frame)
        return base_frame_RCF.transformed(T)

    def get_link_names(self, group=None):
        """Returns the names of the links in the chain.

        Parameters
        ----------
        group : str
            The name of the group. Defaults to `None`.

        Returns
        -------
        list of str

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
        """Returns the configurable joints.

        Parameters
        ----------
        group : str
            The name of the group. Defaults to `None`.

        Returns
        -------
        list of :class: `compas.robots.Joint`

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
                joints = []
                for group in self.group_names:
                    joints_in_group = self.semantics.get_configurable_joints(
                        group)
                    for joint in joints_in_group:
                        if joint not in joints:  # Check to not add double joints
                            joints.append(joint)
                return joints
        else:
            return self.model.get_configurable_joints()

    def get_joint_types_by_names(self, names):
        """Returns a list of joint types for a list of joint names.

        Parameters
        ----------
        name: list of str
            The names of the joints.

        Returns
        -------
        list of str
        """
        return [self.get_joint_by_name(n).type for n in names]

    def get_joint_by_name(self, name):
        """Returns the joint in the robot model matching its name.

        Parameters
        ----------
        name: str
            The name of the joint.

        Returns
        -------
        :class:`compas.robots.Joint`
        """
        return self.model.get_joint_by_name(name)

    def get_configurable_joint_names(self, group=None):
        """Returns the configurable joint names.

        Parameters
        ----------
        group : str
            The name of the group. Defaults to `None`.

        Returns
        -------
        list of str

        Note
        ----
        If semantics is set and no group is passed, it returns all configurable
        joints of all groups.

        Examples
        --------
        >>> robot.get_configurable_joint_names('manipulator')
        ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        """
        configurable_joints = self.get_configurable_joints(group)
        return [j.name for j in configurable_joints]

    def get_configurable_joint_types(self, group=None):
        """Returns the configurable joint types.

        Parameters
        ----------
        group : str
            The name of the group. Defaults to `None`.

        Returns
        -------
        list of int

        Note
        ----
        If semantics is set and no group is passed, it returns all configurable
        joint types of all groups.

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

    def init_configuration(self, group=None):
        """Returns the init joint configuration.

        Examples
        --------
        >>> robot.init_configuration('manipulator')
        Configuration((0.000, 0.000, 0.000, 0.000, 0.000, 0.000), (0, 0, 0, 0, 0, 0))
        """
        types = [joint.type for joint in self.get_configurable_joints(group)]
        positions = [0.] * len(types)
        return Configuration(positions, types)

    def random_configuration(self, group=None):
        """Returns a random configuration.

        Note that no collision checking is involved, so the configuration may be invalid.
        """
        configurable_joints = self.get_configurable_joints(group)
        values = []
        types = [j.type for j in configurable_joints]
        for joint in configurable_joints:
            if joint.limit:
                values.append(joint.limit.lower + (joint.limit.upper - joint.limit.lower) * random.random())
            else:
                values.append(0)
        return Configuration(values, types)

    def get_group_configuration(self, group, full_configuration):
        """Returns the group's configuration.

        Parameters
        ----------
        group : str
            The name of the group.
        full_configuration : :class:`compas_fab.robots.Configuration`
            The configuration for all configurable joints of the robot.

        Returns
        -------
        :class:`compas_fab.robots.Configuration`
            The configuration of the group.
        """
        values = []
        types = []
        group_joint_names = self.get_configurable_joint_names(group)
        for i, name in enumerate(self.get_configurable_joint_names()):
            if name in group_joint_names:
                types.append(full_configuration.types[i])
                values.append(full_configuration.values[i])
        return Configuration(values, types)

    def merge_group_with_full_configuration(self, group_configuration, full_configuration, group):
        """Returns a robot's full configuration by merging a group's configuration with a full configuration.

        Parameters
        ----------
        group_configuration : :class:`compas_fab.robots.Configuration`
            The configuration for one of the robot's planning groups.
        full_configuration : :class:`compas_fab.robots.Configuration`
            The configuration for all configurable joints of the robot.
        group : str
            The name of the group.

        Returns
        -------
        :class:`compas_fab.robots.Configuration`
            A full configuration: with values for all configurable joints.
        """
        all_joint_names = self.get_configurable_joint_names()

        if len(all_joint_names) != len(full_configuration.values):
            raise ValueError("Please pass a full configuration with %d values" % len(all_joint_names))
        elif len(all_joint_names) == len(group_configuration.values):  # group config == full config
            return group_configuration
        else:
            group_joint_names = self.get_configurable_joint_names(group)
            if len(group_joint_names) != len(group_configuration.values):
                raise ValueError('Please pass a group configuration with {} values'.format(len(group_joint_names)))

            configuration = full_configuration.copy()
            for i, name in enumerate(all_joint_names):
                if name in group_joint_names:
                    gi = group_joint_names.index(name)
                    configuration.values[i] = group_configuration.values[gi]
            return configuration

    def get_position_by_joint_name(self, configuration, joint_name, group=None):
        """Returns the value of the joint_name in the passed configuration.
        """
        names = self.get_configurable_joint_names(group)
        if len(names) != len(configuration.values):
            raise ValueError(
                "Please pass a configuration with %d values or specify group" % len(names))
        return configuration.values[names.index(joint_name)]

    def _scale_joint_values(self, values, names, scale_factor, group=None):
        """Scales the scaleable joint values with scale_factor.
        """
        values_scaled = []
        for v, name in zip(values, names):
            j = self.get_joint_by_name(name)
            if j.is_scalable():
                v *= scale_factor
            values_scaled.append(v)
        return values_scaled

    def _get_scaled_joint_positions_from_start_configuration(self, start_configuration=None):
        """Checks the start configuration and returns joint_positions.
        """
        joint_names = self.get_configurable_joint_names()  # full configuration
        joint_positions = [0] * len(joint_names)
        if start_configuration:
            if len(joint_names) != len(start_configuration.values):
                raise ValueError("Please pass a configuration with %d values" % len(joint_names))
            joint_positions = start_configuration.values
        # scale the prismatic joints
        joint_positions = self._scale_joint_values(
            joint_positions, joint_names, 1. / self.scale_factor)
        return joint_positions

    # ==========================================================================
    # transformations, coordinate frames
    # ==========================================================================

    def transformation_RCF_WCF(self, group=None):
        """Returns the transformation from the robot's coordinate system (RCF) to the world coordinate system (WCF).

        Parameters
        ----------
        group : str
            The name of the planning group. Defaults to `None`.

        Returns
        -------
        :class:`compas.geometry.Transformation`

        """
        base_frame = self.get_base_frame(group)
        return Transformation.change_basis(base_frame, Frame.worldXY())

    def _get_current_transformation_WCF_RCF(self, full_configuration, group):
        """Returns the group's current WCF to RCF transformation, if the robot is in full_configuration.

        The base_frame of a planning group can change if a parent joint was
        transformed. This function performs a forward kinematic request with the
        full configuration to retrieve the (possibly) transformed base_frame of
        planning group. This function is only used in plan_motion since other
        services, such as ik or plan_cartesian_motion, do not use the
        transformed base_frame as the group's local coordinate system.

        Parameters
        ----------
        full_configuration : :class:`compas_fab.robots.Configuration`
            The (full) configuration from which the group's base frame is
            calculated.
        group : str
            The planning group for which we want to get the transformed base frame.

        Returns
        -------
        :class:`compas.geometry.Transformation`
        """
        base_frame = self._get_current_base_frame(full_configuration, group)
        return Transformation.from_frame_to_frame(base_frame, Frame.worldXY())

    def transformation_WCF_RCF(self, group=None):
        """Returns the transformation from the world coordinate system (WCF) to the robot's coordinate system (RCF).

        Parameters
        ----------
        group : str
            The name of the planning group. Defaults to `None`.

        Returns
        -------
        :class:`compas.geometry.Transformation`

        """
        base_frame = self.get_base_frame(group)
        return Transformation.change_basis(Frame.worldXY(), base_frame)

    def set_RCF(self, robot_coordinate_frame, group=None):
        """Moves the origin frame of the robot to the robot_coordinate_frame.
        """
        # TODO: must be applied to the model, so that base_frame is RCF
        raise NotImplementedError

    def get_RCF(self, group=None):
        """Returns the origin frame of the robot.
        """
        return self.get_base_frame(group)

    def to_local_coords(self, frame_WCF, group=None):
        """Represents a frame from the world coordinate system (WCF) in the robot's coordinate system (RCF).

        Parameters
        ----------
        frame_WCF : :class:`compas.geometry.Frame`
            A frame in the world coordinate frame.

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
        """Represents a frame from the robot's coordinate system (RCF) in the world coordinate system (WCF).

        Parameters
        ----------
        frame_RCF : :class:`compas.geometry.Frame`
            A frame in the robot's coordinate frame.

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

    def to_t0cf(self, frame_tcf):
        """Converts a frame at the robot's tool tip (tcf frame) to a frame at the robot's flange (tool0 frame) using the set end-effector.

        Parameters
        ----------
        frame_t0cf : :class:`Frame`
            A frame (in WCF) at the robot's tool tip (tcf).

        Returns
        -------
        :class:`Frame`
            A frame (in WCF) at the robot's flange (tool0).

        Raises
        ------
        Exception
            If the end effector is not set.

        Examples
        --------
        >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
        >>> frame = Frame([0.14, 0, 0], [0, 1, 0], [0, 0, 1])
        >>> acm = robot.set_end_effector(mesh, frame)
        >>> frame_tcf = Frame((-0.309, -0.046, -0.266), (0.276, 0.926, -0.256), (0.879, -0.136, 0.456))
        >>> robot.to_t0cf(frame_tcf)
        Frame(Point(-0.363, 0.003, -0.147), Vector(0.388, -0.351, -0.852), Vector(0.276, 0.926, -0.256))
        """
        if not self.end_effector_frame:
            raise Exception("Please set an endeffector")
        Te = Transformation.from_frame_to_frame(self.end_effector_frame, Frame.worldXY())
        Tc = Transformation.from_frame(frame_tcf)
        return Frame.from_transformation(Tc * Te)

    def to_tcf(self, frame_t0cf):
        """Converts a frame at the robot's flange (tool0 frame) to a frame at the robot's tool tip (tcf frame) using the set end-effector.

        Parameters
        ----------
        frame_t0cf : :class:`Frame`
            A frame (in WCF) at the robot's flange (tool0).

        Returns
        -------
        :class:`Frame`
            A frame (in WCF) at the robot's tool tip (tcf).

        Raises
        ------
        Exception
            If the end effector is not set.

        Examples
        --------
        >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
        >>> frame = Frame([0.14, 0, 0], [0, 1, 0], [0, 0, 1])
        >>> acm = robot.set_end_effector(mesh, frame)
        >>> frame_t0cf = Frame((-0.363, 0.003, -0.147), (0.388, -0.351, -0.852), (0.276, 0.926, -0.256))
        >>> robot.to_tcf(frame_t0cf)
        Frame(Point(-0.309, -0.046, -0.266), Vector(0.276, 0.926, -0.256), Vector(0.879, -0.136, 0.456))
        """
        # TODO: or, rather than raising exception: do we simply read the end-effector from the urdf
        if not self.end_effector_frame:
            raise Exception("Please set an endeffector")
        Te = Transformation.from_frame_to_frame(Frame.worldXY(), self.end_effector_frame)
        Tc = Transformation.from_frame(frame_t0cf)
        return Frame.from_transformation(Tc * Te)

    def set_end_effector(self, mesh, frame, group=None, touch_links=None):
        """Set's the robots end-effector that is not defined through URDF.

        Parameters
        ----------
        mesh : :class:`compas.datastructures.Mesh`
            The geometry of the end-effector. Must be placed and oriented such
            that the Frame.worldXY() corresponds to the robot's tool0-frame.
        frame : :class:`compas.geometry.Frame`
            The frame of the end-effector.
        group : str
            The planning group to attach this end-effector to. Defaults to the
            main planning group.
        touch_links : list of str
            A list of link names the end-effector is allowed to touch. Defaults
            to the end-effector link.

        Returns
        -------
        :class:`AttachedCollisionMesh`

        Examples
        --------
        >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
        >>> frame = Frame([0.14, 0, 0], [0, 1, 0], [0, 0, 1])
        >>> acm = robot.set_end_effector(mesh, frame)
        """
        group = group or self.main_group_name
        ee_link_name = self.get_end_effector_link_name(group)
        touch_links = touch_links or [ee_link_name]
        cm = CollisionMesh(mesh, 'end_effector')
        self.end_effector = AttachedCollisionMesh(cm, ee_link_name, touch_links)
        self.end_effector_frame = Frame(frame[0], frame[1], frame[2])
        return self.end_effector

    # ==========================================================================
    # checks
    # ==========================================================================

    def ensure_client(self):
        """Checks if the client is set."""
        if not self.client:
            raise Exception(
                'This method is only callable once a client is assigned')

    def ensure_semantics(self):
        """Checks if semantics is set."""
        if not self.semantics:
            raise Exception(
                'This method is only callable once a semantic model is assigned')

    # ==========================================================================
    # constraints
    # ==========================================================================

    def orientation_constraint_from_frame(self, frame_WCF, tolerances_axes,
                                          group=None):
        """Returns an orientation constraint on the group's end-effector link.

        Parameters
        ----------
        frame_WCF: :class:`compas.geometry.Frame`
            The frame from which we create the orientation constraint.
        tolerances_axes: list of float
            Error tolerances ti for each of the frame's axes in radians. If only
            one value is passed it will be uses for all 3 axes.
        group: str
            The planning group for which we specify the constraint. Defaults to
            the robot's main planning group.

        Examples
        --------
        >>> frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
        >>> tolerances_axes = [math.radians(1)] * 3
        >>> group = robot.main_group_name
        >>> robot.orientation_constraint_from_frame(frame, tolerances_axes, group=group)
        OrientationConstraint('ee_link', [0.5, 0.5, 0.5, 0.5], [0.017453292519943295, 0.017453292519943295, 0.017453292519943295], 1.0)

        Notes
        -----
        If you specify the tolerances_axes vector with [0.01, 0.01, 6.3], it
        means that the frame's x-axis and y-axis are allowed to rotate about the
        z-axis by an angle of 6.3 radians, whereas the z-axis would only rotate
        by 0.01.
        """

        ee_link = self.get_end_effector_link_name(group)

        tolerances_axes = list(tolerances_axes)
        if len(tolerances_axes) == 1:
            tolerances_axes *= 3
        elif len(tolerances_axes) != 3:
            raise ValueError("Must give either one or 3 values")
        return OrientationConstraint(ee_link, frame_WCF.quaternion, tolerances_axes)

    def position_constraint_from_frame(self, frame_WCF, tolerance_position, group=None):
        """Returns a position and orientation constraint on the group's end-effector link.

        Parameters
        ----------
        frame_WCF : :class:`compas.geometry.Frame`
            The frame from which we create position and orientation constraints.
        tolerance_position : float
            The allowed tolerance to the frame's position. (Defined in the
            robot's units)
        group: str
            The planning group for which we specify the constraint. Defaults to
            the robot's main planning group.

        Examples
        --------
        >>> frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
        >>> tolerance_position = 0.001
        >>> robot.position_constraint_from_frame(frame, tolerance_position)
        PositionConstraint('ee_link', BoundingVolume(2, Sphere(Point(0.400, 0.300, 0.400), 0.001)), 1.0)

        Notes
        -----
        There are many other possibilities of how to create a position and
        orientation constraints. Checkout :class:`compas_fab.robots.PositionConstraint`
        and :class:`compas_fab.robots.OrientationConstraint`.

        """

        ee_link = self.get_end_effector_link_name(group)
        sphere = Sphere(frame_WCF.point, tolerance_position)
        return PositionConstraint.from_sphere(ee_link, sphere)

    def constraints_from_frame(self, frame_WCF, tolerance_position, tolerances_axes, group=None):
        """Returns a position and orientation constraint on the group's end-effector link.

        Parameters
        ----------
        frame_WCF: :class:`compas.geometry.Frame`
            The frame from which we create position and orientation constraints.
        tolerance_position: float
            The allowed tolerance to the frame's position. (Defined in the
            robot's units)
        tolerances_axes: list of float
            Error tolerances ti for each of the frame's axes in radians. If only
            one value is passed it will be uses for all 3 axes.
        group: str
            The planning group for which we specify the constraint. Defaults to
            the robot's main planning group.

        Examples
        --------
        >>> frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
        >>> tolerance_position = 0.001
        >>> tolerances_axes = [math.radians(1)]
        >>> group = robot.main_group_name
        >>> robot.constraints_from_frame(frame, tolerance_position, tolerances_axes, group)
        [PositionConstraint('ee_link', BoundingVolume(2, Sphere(Point(0.400, 0.300, 0.400), 0.001)), 1.0), \
        OrientationConstraint('ee_link', [0.5, 0.5, 0.5, 0.5], [0.017453292519943295, 0.017453292519943295, 0.017453292519943295], 1.0)]

        Notes
        -----
        There are many other possibilities of how to create a position and
        orientation constraint. Checkout :class:`compas_fab.robots.PositionConstraint`
        and :class:`compas_fab.robots.OrientationConstraint`.

        """
        pc = self.position_constraint_from_frame(frame_WCF, tolerance_position, group)
        oc = self.orientation_constraint_from_frame(frame_WCF, tolerances_axes, group)
        return [pc, oc]

    def constraints_from_configuration(self, configuration, tolerances, group=None):
        """Returns joint constraints on all joints of the configuration.

        Parameters
        ----------
        configuration: :class:`compas_fab.robots.Configuration`
            The target configuration.
        tolerances: list of float
            The tolerances (as +/-) on each of the joints defining the bound in radian
            to be achieved. If only one value is passed it will be used to create
            bounds for all joint constraints.
        group: str, optional
            The planning group for which we specify the constraint. Defaults to
            the robot's main planning group.

        Examples
        --------
        >>> configuration = Configuration.from_revolute_values([-0.042, 4.295, -4.110, -3.327, 4.755, 0.])
        >>> tolerances = [math.radians(5)] * 6
        >>> group = robot.main_group_name
        >>> robot.constraints_from_configuration(configuration, tolerances, group)
        [JointConstraint('shoulder_pan_joint', -0.042, 0.08726646259971647, 1.0), \
        JointConstraint('shoulder_lift_joint', 4.295, 0.08726646259971647, 1.0), \
        JointConstraint('elbow_joint', -4.11, 0.08726646259971647, 1.0), \
        JointConstraint('wrist_1_joint', -3.327, 0.08726646259971647, 1.0), \
        JointConstraint('wrist_2_joint', 4.755, 0.08726646259971647, 1.0), \
        JointConstraint('wrist_3_joint', 0.0, 0.08726646259971647, 1.0)]

        Raises
        ------
        ValueError
            If the passed configuration does not correspond to the group.
        ValueError
            If the passed tolerances have a different length than the configuration.

        Notes
        -----
        Check for using the correct tolerance units for prismatic and revolute
        joints.

        """
        if not group:
            group = self.main_group_name

        joint_names = self.get_configurable_joint_names(group)
        if len(joint_names) != len(configuration.values):
            raise ValueError("The passed configuration has %d values, the group %s needs however: %d" % (
                len(configuration.values), group, len(joint_names)))
        if len(tolerances) == 1:
            tolerances = tolerances * len(joint_names)
        elif len(tolerances) != len(configuration.values):
            raise ValueError("The passed configuration has %d values, the tolerances however: %d" % (
                len(configuration.values), len(tolerances)))

        constraints = []
        for name, value, tolerance in zip(joint_names, configuration.values, tolerances):
            constraints.append(JointConstraint(name, value, tolerance))
        return constraints

    # ==========================================================================
    # services
    # ==========================================================================

    def inverse_kinematics(self, frame_WCF, start_configuration=None,
                           group=None, avoid_collisions=True,
                           constraints=None, attempts=8,
                           attached_collision_meshes=None):
        """Calculate the robot's inverse kinematic for a given frame.

        Parameters
        ----------
        frame: :class:`compas.geometry.Frame`
            The frame to calculate the inverse for.
        start_configuration: :class:`compas_fab.robots.Configuration`, optional
            If passed, the inverse will be calculated such that the calculated
            joint positions differ the least from the start_configuration.
            Defaults to the init configuration.
        group: str, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        avoid_collisions: bool, optional
            Whether or not to avoid collisions. Defaults to True.
        constraints: list of :class:`compas_fab.robots.Constraint`, optional
            A set of constraints that the request must obey. Defaults to None.
        attempts: int, optional
            The maximum number of inverse kinematic attempts. Defaults to 8.
        attached_collision_meshes: list of :class:`compas_fab.robots.AttachedCollisionMesh`
            Defaults to None.

        Raises
        ------
        compas_fab.backends.exceptions.BackendError
            If no configuration can be found.

        Returns
        -------
        :class:`compas_fab.robots.Configuration`
            The planning group's configuration.

        Examples
        --------
        >>> frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
        >>> start_configuration = robot.init_configuration()
        >>> group = robot.main_group_name
        >>> robot.inverse_kinematics(frame_WCF, start_configuration, group)                 # doctest: +SKIP
        Configuration((4.045, 5.130, -2.174, -6.098, -5.616, 6.283), (0, 0, 0, 0, 0, 0))    # doctest: +SKIP
        """
        self.ensure_client()
        if not group:
            group = self.main_group_name  # ensure semantics
        base_link = self.get_base_link_name(group)
        joint_names = self.get_configurable_joint_names()

        joint_positions = self._get_scaled_joint_positions_from_start_configuration(
            start_configuration)

        # represent in RCF
        frame_RCF = self.to_local_coords(frame_WCF, group)
        frame_RCF.point /= self.scale_factor  # must be in meters

        if self.end_effector:
            attached_collision_meshes = attached_collision_meshes or []
            attached_collision_meshes.append(self.end_effector)

        # The returned joint names might be more than the requested ones if there are passive joints present
        joint_positions, joint_names = self.client.inverse_kinematics(frame_RCF, base_link,
                                                                      group, joint_names, joint_positions,
                                                                      avoid_collisions, constraints, attempts,
                                                                      attached_collision_meshes)
        joint_types = [self.get_joint_by_name(n).type for n in joint_names]

        joint_positions = self._scale_joint_values(joint_positions, joint_names, self.scale_factor)

        # build configuration including passive joints
        configuration = Configuration(joint_positions, joint_types)

        # return only group configuration
        # NOTE: it might actually make more sense to return
        # the configuration instance without extracting the group's config
        # because we lose the passive joint info
        return self.get_group_configuration(group, configuration)

    def forward_kinematics(self, configuration, group=None, backend=None, link_name=None):
        """Calculate the robot's forward kinematic.

        Parameters
        ----------
        configuration : :class:`compas_fab.robots.Configuration`
            The configuration to calculate the forward kinematic for.
        group : str, optional
            The planning group used for the calculation. Defaults to the robot's
            main planning group.
        backend : None or str
            If `None` calculates fk with the client if it exists or with the robot model.
            If 'model' use the robot model to calculate fk. Anything else is open
            for implementation, possibly 'kdl', 'ikfast'
        link_name : str, optional
            The name of the link to calculate the forward kinematics for.
            Defaults to the group's end effector link.

        Returns
        -------
        :class:`Frame`
            The frame in the robot's coordinate system (RCF).

        Examples
        --------
        >>> configuration = Configuration.from_revolute_values([-2.238, -1.153, -2.174, 0.185, 0.667, 0.000])
        >>> group = robot.main_group_name
        >>> frame_RCF_c = robot.forward_kinematics(configuration, group)
        >>> frame_RCF_m = robot.forward_kinematics(configuration, group, backend='model')
        >>> frame_RCF_c == frame_RCF_m
        True
        >>> frame_WCF = robot.to_world_coords(frame_RCF_m, group)
        >>> frame_WCF
        Frame(Point(0.300, 0.100, 0.500), Vector(1.000, -0.000, -0.000), Vector(0.000, 1.000, -0.000))

        """
        if not group:
            group = self.main_group_name

        if link_name is None:
            link_name = self.get_end_effector_link_name(group)
        else:
            # check
            if link_name not in self.get_link_names(group):
                raise ValueError("Link name %s does not exist in planning group" % link_name)

        full_configuration = self.merge_group_with_full_configuration(configuration, self.init_configuration(), group)
        full_joint_positions = self._get_scaled_joint_positions_from_start_configuration(full_configuration)
        full_joint_names = self.get_configurable_joint_names()
        base_link_name = self.get_base_link_name(group)

        group_configuration = self.get_group_configuration(group, full_configuration)
        group_joint_names = self.get_configurable_joint_names(group)
        group_joint_state = dict(zip(group_joint_names, group_configuration.values))  # assuming configuration is group configuration

        if not backend:
            if self.client:
                frame_RCF = self.client.forward_kinematics(full_joint_positions, base_link_name, group, full_joint_names, link_name)
                frame_RCF.point *= self.scale_factor
            else:
                frame_WCF = self.model.forward_kinematics(group_joint_state, link_name)
                frame_RCF = self.to_local_coords(frame_WCF, group)
        elif backend == 'model':
            frame_WCF = self.model.forward_kinematics(group_joint_state, link_name)
            frame_RCF = self.to_local_coords(frame_WCF, group)
        else:
            # pass to backend, kdl, ikfast,...
            raise NotImplementedError

        return frame_RCF

    def forward_kinematics_robot_model(self, configuration, group=None, link_name=None):
        """Calculate the robot's forward kinematic with the robot model.

        Parameters
        ----------
        configuration : :class:`compas_fab.robots.Configuration`
            The configuration to calculate the forward kinematic for.
        group : str, optional
            The planning group used for the calculation. Defaults to the robot's
            main planning group.
        link_name : str
            The name of the link to calculate the forward kinematics for.
            Defaults to the group's end effector link.

        Examples
        --------
        >>> configuration = Configuration.from_revolute_values([-2.238, -1.153, -2.174, 0.185, 0.667, 0.000])
        >>> group = robot.main_group_name
        >>> frame_WCF = robot.forward_kinematics(configuration, group)
        >>> frame_WCF
        Frame(Point(0.300, 0.100, 0.500), Vector(1.000, -0.000, -0.000), Vector(0.000, 1.000, -0.000))
        """
        if link_name is None:
            link_name = self.get_end_effector_link_name(group)

        joint_names = self.get_configurable_joint_names(group)
        joint_state = dict(zip(joint_names, configuration.values))
        return self.model.forward_kinematics(joint_state, link_name)

    def plan_cartesian_motion(self, frames_WCF, start_configuration=None,
                              max_step=0.01, jump_threshold=1.57,
                              avoid_collisions=True, group=None,
                              path_constraints=None,
                              attached_collision_meshes=None):
        """Calculates a cartesian motion path (linear in tool space).

        Parameters
        ----------
        frames_WCF: list of :class:`compas.geometry.Frame`
            The frames through which the path is defined.
        start_configuration: :class:`Configuration`, optional
            The robot's configuration at the starting position. Defaults to the
            zero configuration.
        max_step: float
            The approximate distance between the calculated points. (Defined in
            the robot's units)
        jump_threshold: float
            The maximum allowed distance of joint positions between consecutive
            points. If the distance is found to be above this threshold, the
            path computation fails. It must be specified in relation to max_step.
            If this theshhold is 0, 'jumps' might occur, resulting in an invalid
            cartesian path. Defaults to pi/2.
        avoid_collisions: bool, optional
            Whether or not to avoid collisions. Defaults to True.
        group: str, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        path_constraints: list of :class:`compas_fab.robots.Constraint`, optional
            Optional constraints that can be imposed along the solution path.
            Note that path calculation won't work if the start_configuration
            violates these constraints. Defaults to None.
        attached_collision_meshes: list of :class:`compas_fab.robots.AttachedCollisionMesh`
            Defaults to None.

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
        frames_RCF = []
        for frame_WCF in frames_WCF:
            # represent in RCF
            frame_RCF = self.to_local_coords(frame_WCF, group)
            frame_RCF.point /= self.scale_factor
            frames_RCF.append(frame_RCF)

        start_configuration = start_configuration.copy() if start_configuration else self.init_configuration()
        start_configuration.scale(1. / self.scale_factor)
        max_step_scaled = max_step / self.scale_factor

        T = self.transformation_WCF_RCF(group)
        if path_constraints:
            path_constraints_RCF_scaled = []
            for c in path_constraints:
                cp = c.copy()
                cp.transform(T)
                if c.type == Constraint.JOINT:
                    joint = self.get_joint_by_name(c.joint_name)
                    if joint.is_scalable():
                        cp.scale(self.scale_factor)
                else:
                    cp.scale(self.scale_factor)
                path_constraints_RCF_scaled.append(cp)
        else:
            path_constraints_RCF_scaled = None
    
        if self.end_effector:
            attached_collision_meshes = attached_collision_meshes or []
            attached_collision_meshes.append(self.end_effector)

        trajectory = self.client.plan_cartesian_motion(
            robot=self,
            frames=frames_RCF,
            start_configuration=start_configuration,
            group=group,
            max_step=max_step_scaled,
            jump_threshold=jump_threshold,
            avoid_collisions=avoid_collisions,
            path_constraints=path_constraints_RCF_scaled,
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
        """Calculates a motion path.

        Parameters
        ----------
        goal_constraints: list of :class:`compas_fab.robots.Constraint`
            The goal to be achieved, defined in a set of constraints.
            Constraints can be very specific, for example defining value domains
            for each joint, such that the goal configuration is included,
            or defining a volume in space, to which a specific robot link (e.g.
            the end-effector) is required to move to.
        start_configuration: :class:`compas_fab.robots.Configuration`, optional
            The robot's configuration at the starting position. Defaults to the
            all-zero configuration.
        group: str, optional
            The name of the group to plan for. Defaults to the robot's main
            planning group.
        path_constraints: list of :class:`compas_fab.robots.Constraint`, optional
            Optional constraints that can be imposed along the solution path.
            Note that path calculation won't work if the start_configuration
            violates these constraints. Defaults to None.
        planner_id: str
            The name of the algorithm used for path planning. Defaults to 'RRT'.
        num_planning_attempts: int, optional
            Normally, if one motion plan is needed, one motion plan is computed.
            However, for algorithms that use randomization in their execution
            (like 'RRT'), it is likely that different planner executions will
            produce different solutions. Setting this parameter to a value above
            1 will run many additional motion plans, and will report the
            shortest solution as the final result. Defaults to 1.
        allowed_planning_time: float
            The number of seconds allowed to perform the planning. Defaults to 2.
        max_velocity_scaling_factor: float
            Defaults to 1.
        max_acceleration_scaling_factor: float
            Defaults to 1.
        attached_collision_meshes: list of :class:`compas_fab.robots.AttachedCollisionMesh`
            Defaults to None.


        Examples
        --------
        >>> # Example with position and orientation constraints
        >>> frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
        >>> tolerance_position = 0.001
        >>> tolerances_axes = [math.radians(1)] * 3
        >>> start_configuration = Configuration.from_revolute_values([-0.042, 4.295, 0, -3.327, 4.755, 0.])
        >>> group = robot.main_group_name
        >>> goal_constraints = robot.constraints_from_frame(frame, tolerance_position, tolerances_axes, group)
        >>> trajectory = robot.plan_motion(goal_constraints, start_configuration, group, planner_id='RRT')
        >>> trajectory.fraction
        1.0
        >>> # Example with joint constraints (to the UP configuration)
        >>> configuration = Configuration.from_revolute_values([0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0])
        >>> tolerances = [math.radians(5)] * 6
        >>> group = robot.main_group_name
        >>> goal_constraints = robot.constraints_from_configuration(configuration, tolerances, group)
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

        start_configuration = start_configuration.copy() if start_configuration else self.init_configuration()

        # Transform goal constraints to RCF and scale
        full_configuration = self.merge_group_with_full_configuration(start_configuration, self.init_configuration(), group)

        T = self._get_current_transformation_WCF_RCF(full_configuration, group)
        goal_constraints_RCF_scaled = []
        for c in goal_constraints:
            cp = c.copy()
            cp.transform(T)
            if c.type == Constraint.JOINT:
                joint = self.get_joint_by_name(c.joint_name)
                if joint.is_scalable():
                    cp.scale(self.scale_factor)
            else:
                cp.scale(self.scale_factor)
            goal_constraints_RCF_scaled.append(cp)

        # Transform path constraints to RCF and scale
        if path_constraints:
            path_constraints_RCF_scaled = []
            for c in path_constraints:
                cp = c.copy()
                cp.transform(T)
                if c.type == Constraint.JOINT:
                    joint = self.get_joint_by_name(c.joint_name)
                    if joint.is_scalable():
                        cp.scale(self.scale_factor)
                else:
                    cp.scale(self.scale_factor)
                path_constraints_RCF_scaled.append(cp)
        else:
            path_constraints_RCF_scaled = None

        start_configuration.scale(1. / self.scale_factor)

        if self.end_effector:
            attached_collision_meshes = attached_collision_meshes or []
            attached_collision_meshes.append(self.end_effector)

        trajectory = self.client.plan_motion(
            robot=self,
            goal_constraints=goal_constraints_RCF_scaled,
            start_configuration=start_configuration,
            group=group,
            path_constraints=path_constraints_RCF_scaled,
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
        """Returns the robot's transformed frames."""
        joint_names = self.get_configurable_joint_names(group)
        joint_state = dict(zip(joint_names, configuration.values))
        return self.model.transformed_frames(joint_state)

    def transformed_axes(self, configuration, group=None):
        """Returns the robot's transformed axes."""
        joint_names = self.get_configurable_joint_names(group)
        joint_state = dict(zip(joint_names, configuration.values))
        return self.model.transformed_axes(joint_state)

    # ==========================================================================
    # drawing
    # ==========================================================================

    def update(self, configuration, group=None, visual=True, collision=True):
        """Updates the robot's geometry.

        Parameters
        ----------
        configuration : :class:`compas_fab.robots.Configuration`
            Instance of the configuration (joint state) to move to.
        group: str, optional
            The name of the group to plan for. Defaults to the robot's main
            planning group.
        visual : bool, optional
            ``True`` if the visual geometry should be also updated, otherwise ``False``.
            Defaults to ``True``.
        collision : bool, optional
            ``True`` if the collision geometry should be also updated, otherwise ``False``.
            Defaults to ``True``.
        """
        names = self.get_configurable_joint_names(group)
        self.artist.update(configuration, names, visual, collision)

    def draw_visual(self):
        """Draws the visual geometry of the robot in the respective CAD environment.
        """
        return self.artist.draw_visual()

    def draw_collision(self):
        """Draws the collision geometry of the robot in the respective CAD environment.
        """
        return self.artist.draw_collision()

    def draw(self):
        """Draws the visual geometry of the robot in the respective CAD environment.
        """
        return self.draw_visual()

    def draw_end_effector(self, tool0_frame):
        """Draws the end-effector if set.

        Parameters
        ----------
        tool0_frame : :class:`Frame`
            The frame at the robot's flange (tool0).
        """
        if self.end_effector:
            T = Transformation.from_frame_to_frame(self.end_effector.collision_mesh.frame, tool0_frame)
            ee_mesh = mesh_transformed(self.end_effector.collision_mesh.mesh, T)
            return self.artist.draw_geometry(ee_mesh)

    def scale(self, factor):
        """Scales the robot geometry by factor (absolute).

        Parameters
        ----------
        factor : float
            The factor to scale the robot with.

        Returns
        -------
        None
        """
        self.model.scale(factor)
        if self.artist:
            self.artist.scale(factor)
        else:
            self._scale_factor = factor

    @property
    def scale_factor(self):
        """The robot's scale factor."""
        if self.artist:
            return self.artist.scale_factor
        else:
            return self._scale_factor

    def info(self):
        """Prints information about the robot.
        """
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
