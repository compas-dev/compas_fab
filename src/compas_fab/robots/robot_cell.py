import random

from compas import IPY
from compas.data import Data
from compas.geometry import Frame
from compas.geometry import Transformation
from compas_robots import Configuration
from compas_robots import RobotModel
from compas_robots import ToolModel
from compas_robots.model import Joint
from compas_robots.resources import LocalPackageMeshLoader

import compas_fab

from .rigid_body import RigidBody
from .semantics import RobotSemantics
from .targets import TargetMode

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Dict  # noqa: F401
        from typing import List  # noqa: F401
        from typing import Optional  # noqa: F401

        from compas.datastructures import Mesh  # noqa: F401
        from compas_robots.model import Link  # noqa: F401

        from .state import RobotCellState  # noqa: F401
__all__ = [
    "RobotCell",
]


class RobotCell(Data):
    """Represents all objects in a robot cell.
    This include
    - One robot (and its semantics)
    - ToolModels (including attached tools, and tools that are statically placed in the environment)
    - RigidBodies (including attached rigid bodies, and rigid bodies that are statically placed in the environment)

    Rigid bodies are objects can be used to represent:
    - Workpieces that are attached to the tip of a tool
    - Workpieces that are placed in the environment and are not attached
    - Robotic backpacks or other accessories that are attached to links of the robot
    - Static obstacles in the environment

    Notes regarding scaling: All models in the RobotCell should be in meters. If user models are in millimeters,
    they should be scaled to meters before being added to the RobotCell.


    Attributes
    ----------
    robot_model : :class:`~compas_robots.RobotModel`
        The robot model in the robot cell.
        Note that there can only be one robot in the robot cell.
        This is equivalent to the URDF in ROS MoveIt workflows.
    robot_semantics : :class:`~compas_fab.robots.RobotSemantics`
        Semantics describing planning groups, disabled collisions, pre-defined poses etc.
        for the robot model.
        This is equivalent to the SRDF in ROS MoveIt workflows.
    tool_models : dict of str and :class:`~compas_robots.ToolModel`
        The tools in the robot cell.
        The key is the unique identifier for the tool.
    rigid_body_models : dict of str and :class:`~compas_fab.robots.RigidBody`
        The rigid bodies in the robot cell.
        The key is the unique identifier for the rigid body.
    """

    def __init__(self, robot_model=None, robot_semantics=None, tool_models=None, rigid_body_models=None):
        super(RobotCell, self).__init__()
        self.robot_model = robot_model  # type: RobotModel
        self.robot_semantics = robot_semantics  # type: RobotSemantics
        self.tool_models = tool_models or {}  # type: Dict[str, ToolModel]
        self.rigid_body_models = rigid_body_models or {}  # type: Dict[str, RigidBody]

    @property
    def tool_ids(self):
        # type: () -> List[str]
        return self.tool_models.keys()

    @property
    def rigid_body_ids(self):
        # type: () -> List[str]
        return self.rigid_body_models.keys()

    @property
    def __data__(self):
        """Returns the data dictionary that represents the RobotCell.

        The RobotCell can be used as a data storage container in advanced use scenarios for serializing
        a group of tools and environment models.
        In this case, the `robot_model` and `robot_semantics` can be None.
        However, the robot_model and robot_semantics are required for all the planning functions.
        """

        tool_models = {id: tool.__data__ for id, tool in self.tool_models.items()}
        rigid_body_models = {id: rigid_body.__data__ for id, rigid_body in self.rigid_body_models.items()}
        return {
            "robot_model": self.robot_model.__data__ if self.robot_model else None,
            "robot_semantics": self.robot_semantics.__data__ if self.robot_semantics else None,
            "tool_models": tool_models,
            "rigid_body_models": rigid_body_models,
        }

        # NOTE: `"robot_model": self.robot_model` would be more elegant,
        # but it is not working because of the ProxyObject deepcopy failure problem.

    @classmethod
    def __from_data__(cls, data):
        # type: (Dict) -> RobotCell
        """Construct a RobotCell from a data dictionary.

        Parameters
        ----------
        data : dict
            The data dictionary.

        Returns
        -------
        :class:`compas_fab.robots.RobotCell`
            The robot cell.
        """
        robot_model = RobotModel.__from_data__(data["robot"]) if data["robot"] else None
        robot_semantics = RobotSemantics.__from_data__(data["robot_semantics"]) if data["robot_semantics"] else None
        tool_models = {id: ToolModel.__from_data__(tool_data) for id, tool_data in data["tool_models"].items()}
        rigid_body_models = {
            id: RigidBody.__from_data__(rigid_body_data) for id, rigid_body_data in data["rigid_body_models"].items()
        }
        return cls(robot_model, robot_semantics, tool_models, rigid_body_models)

    @classmethod
    def from_urdf_and_srdf(cls, urdf_filename, srdf_filename, local_package_mesh_folder=None):
        # type: (str, Optional[str], Optional[str]) -> RobotCell
        """Create a robot cell from URDF and SRDF files.
        Optionally, a local package mesh folder to load mesh geometry.

        The new RobotCell will not have any tools or rigid bodies.

        Parameters
        ----------
        urdf_filename : :obj:`str`
            Path to the URDF file.
        srdf_filename : :obj:`str`
            Path to the SRDF file to load semantics.
        local_package_mesh_folder : :obj:`str`, optional
            Path to the local package mesh folder.
            If the path is provided, the geometry of the robot is loaded from this folder.
            Default is `None`, which means that the geometry is not loaded.

        Returns
        -------
        :class:`compas_fab.robots.Robot`
            Newly created instance of the robot.

        """
        robot_model = RobotModel.from_urdf_file(urdf_filename)  # type: RobotModel

        robot_semantics = RobotSemantics.from_srdf_file(srdf_filename, robot_model)

        if local_package_mesh_folder:
            loader = LocalPackageMeshLoader(compas_fab.get(local_package_mesh_folder), "")
            robot_model.load_geometry(loader)

        robot_cell = cls(robot_model, robot_semantics)

        return robot_cell

    # ------------------
    # Consistency checks
    # ------------------

    def assert_cell_state_match(self, robot_cell_state):
        # type: (RobotCellState) -> None
        """Assert that the number of tools and rigid bodies in the cell state match the number of tools and workpieces in the robot cell."""
        symmetric_difference = set(robot_cell_state.tool_ids) ^ set(self.tool_ids)
        if symmetric_difference != set():
            raise ValueError(
                "The tools in the cell state do not match the tools in the robot cell. Mismatch: %s"
                % symmetric_difference
            )
        symmetric_difference = set(robot_cell_state.rigid_body_ids) ^ set(self.rigid_body_ids)
        if symmetric_difference != set():
            raise ValueError(
                "The workpieces in the cell state do not match the workpieces in the robot cell. Mismatch: %s"
                % symmetric_difference
            )

    def ensure_semantics(self):
        # type: () -> None
        """Check if semantics is set.

        Raises
        ------
        :exc:`Exception`
            If :attr:`semantics` is not set.
        """
        if not self.robot_semantics:
            raise Exception("Robot semantic is not assigned in the RobotCell.")

    def ensure_geometry(self):
        # type: () -> None
        """Check if the model's geometry has been loaded.

        Raises
        ------
        :exc:`Exception`
            If geometry has not been loaded.
        """
        try:
            self.robot_model.ensure_geometry()
        except Exception:
            raise Exception("RobotModel in the RobotCell has no geometry loaded.")

    # -------------------------------------------------
    # Functions related robot_model and robot_semantics
    # -------------------------------------------------
    @property
    def root_name(self):
        """Robot's root name."""
        return self.robot_model.root.name

    @property
    def main_group_name(self):
        # type: () -> str
        """
        Robot's main planning group, only available if semantics is set.

        Returns
        -------
        :obj:`str`

        """
        self.ensure_semantics()
        return self.robot_semantics.main_group_name

    @property
    def group_names(self):
        # type: () -> List[str]
        """All planning groups of the robot, only available if semantics is set.

        Returns
        -------
        :obj:`list` of :obj:`str`

        Examples
        --------
        >>> robot_cell, robot_cell_state = RobotCellLibrary.ur5()
        >>> robot_cell.group_names
        ['manipulator', 'endeffector']

        """
        self.ensure_semantics()
        return self.robot_semantics.group_names

    @property
    def group_states(self):
        # type: () -> Dict[str, dict]
        """All group states of the robot, only available if semantics is set.

        Returns
        -------
        :obj:`dict` of :obj:`dict` of :obj:`dict` of :obj:`float`
            The first key is the group name, the second key is the group state name.
            At this level you get a joint dictionary.
            The third key is the joint name, and the value is the joint value.

        Examples
        --------
        >>> robot_cell, robot_cell_state = RobotCellLibrary.ur5()
        >>> sorted(robot_cell.group_states['manipulator'].keys())
        ['home', 'up']

        """
        self.ensure_semantics()
        return self.robot_semantics.group_states

    def get_end_effector_link_name(self, group=None):
        # type: (Optional[str]) -> str
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
        >>> robot_cell, robot_cell_state = RobotCellLibrary.ur5()
        >>> robot_cell.get_end_effector_link_name()
        'tool0'
        """
        return self.robot_semantics.get_end_effector_link_name(group or self.main_group_name)

    def get_end_effector_link(self, group=None):
        # type: (Optional[str]) -> Link
        """Get the robot's end effector link.

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`compas_robots.Link`

        Examples
        --------
        >>> robot_cell, robot_cell_state = RobotCellLibrary.ur5()
        >>> link = robot_cell.get_end_effector_link()
        >>> link.name
        'tool0'
        """
        name = self.get_end_effector_link_name(group or self.main_group_name)
        return self.robot_model.get_link_by_name(name)

    def get_base_link_name(self, group=None):
        # type: (Optional[str]) -> str
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
        >>> robot_cell, robot_cell_state = RobotCellLibrary.ur5()
        >>> robot_cell.get_base_link_name()
        'base_link'
        """
        return self.robot_semantics.get_base_link_name(group or self.main_group_name)

    def get_base_link(self, group=None):
        # type: (Optional[str]) -> Link
        """Get the robot's base link.

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`compas_robots.Link`

        Examples
        --------
        >>> robot_cell, robot_cell_state = RobotCellLibrary.ur5()
        >>> link = robot_cell.get_base_link()
        >>> link.name
        'base_link'
        """
        name = self.get_base_link_name(group or self.main_group_name)
        return self.robot_model.get_link_by_name(name)

    def get_link_names(self, group=None):
        # type: (Optional[str]) -> List[str]
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
        >>> robot_cell, robot_cell_state = RobotCellLibrary.ur5()
        >>> robot_cell.get_link_names('manipulator')
        ['base_link', 'base_link_inertia', 'shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'flange', 'tool0']
        """
        group = group or self.main_group_name
        base_link_name = self.get_base_link_name(group)
        end_effector_link_name = self.get_end_effector_link_name(group)
        link_names = []
        for link in self.robot_model.iter_link_chain(base_link_name, end_effector_link_name):
            link_names.append(link.name)
        return link_names

    def get_link_names_with_collision_geometry(self):
        # type: () -> List[str]
        """Get the names of the links with collision geometry.

        Note that returned names does not imply that the link has collision geometry loaded.
        Use :meth:`ensure_geometry()` to ensure that collision geometry is loaded.

        Returns
        -------
        :obj:`list` of :obj:`str`

        Examples
        --------
        >>> robot_cell, robot_cell_state = RobotCellLibrary.ur5()
        >>> robot_cell.get_link_names_with_collision_geometry()
        ['base_link_inertia', 'shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']
        """
        return [link.name for link in self.robot_model.iter_links() if link.collision]

    def get_all_configurable_joints(self):
        # type: () -> List[Joint]
        """Get all configurable :class:`compas_robots.model.Joint` of the robot.

        Configurable joints are joints that can be controlled,
        i.e., not ``Joint.FIXED``, not mimicking another joint and not a passive joint.
        See :meth:`compas_robots.model.Joint.is_configurable` for more details.

        Returns
        -------
        :obj:`list` of :class:`compas_robots.model.Joint`
            A list of configurable joints.

        """

        joints = []
        for joint in self.robot_model.get_configurable_joints():
            if joint.name not in self.robot_semantics.passive_joints:
                joints.append(joint)
        return joints

    def get_configurable_joints(self, group=None):
        # type: (Optional[str]) -> List[Joint]
        """Get all configurable :class:`compas_robots.model.Joint` of a planning group.

        Configurable joints are joints that can be controlled,
        i.e., not ``Joint.FIXED``, not mimicking another joint and not a passive joint.
        See :meth:`compas_robots.model.Joint.is_configurable` for more details.

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :obj:`list` of :class:`compas_robots.model.Joint`
            A list of configurable joints.

        """
        group = group or self.main_group_name
        joints = []
        for name in self.robot_semantics.groups[group]["joints"]:
            joint = self.robot_model.get_joint_by_name(name)  # type: Joint
            if joint:
                if joint.is_configurable() and name not in self.robot_semantics.passive_joints:
                    joints.append(joint)
        return joints

    def get_configurable_joint_names(self, group=None):
        # type: (RobotModel, Optional[str]) -> List[str]
        """Get all the names of configurable joints of a planning group.

        Similar to :meth:`get_configurable_joints` but returning joint names.

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :obj:`list` of :obj:`str`
        """
        group = group or self.main_group_name
        configurable_joints = self.get_configurable_joints(group)
        return [joint.name for joint in configurable_joints]

    def get_configurable_joint_types(self, group=None):
        # type: (Optional[str]) -> List[int]
        """Get the configurable joint types.

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :obj:`list` of :attr:`compas_robots.Joint.SUPPORTED_TYPES`

        Notes
        -----
        If :attr:`semantics` is set and no group is passed, it returns all
        configurable joint types of all groups.

        Examples
        --------
        >>> robot_cell, robot_cell_state = RobotCellLibrary.ur5()
        >>> robot_cell.get_configurable_joint_types('manipulator')
        [0, 0, 0, 0, 0, 0]
        """
        group = group or self.main_group_name
        configurable_joints = self.get_configurable_joints(group)
        return [j.type for j in configurable_joints]

    def get_configuration_from_group_state(self, group, group_state):
        # type: (str, str) -> Configuration
        """Get the :class:`compas_robots.Configuration` from a predefined group state.

        Group states are predefined configurations of a planning group in the RobotSemantics.

        Parameters
        ----------
        group : :obj:`str`
            The name of the planning group.
        group_state : :obj:`str`
            The name of the group_state.

        Returns
        -------
        :class:`compas_robots.Configuration`
            The configuration specified by the :attr:`group_state`.
        """
        joint_dict = self.group_states[group][group_state]
        group_joint_names = self.get_configurable_joint_names(group)
        group_joint_types = self.get_configurable_joint_types(group)
        values = [joint_dict[name] for name in group_joint_names]
        return Configuration(values, group_joint_types, group_joint_names)

    # --------------------
    # Robot Configurations
    # --------------------

    def zero_full_configuration(self):
        # type: () -> Configuration
        """Get the zero configuration (all configurable joints) of the robot.

        If the joint value `0.0` is outside of joint limits ``(joint.limit.upper, joint.limit.lower)`` then
        ``(upper + lower) / 2`` is used as joint value.

        Returns
        -------
        :class:`compas_robots.Configuration`
            The zero configuration of the robot.

        """
        joint_values = []
        joint_names = []
        joint_types = []

        configurable_joints = self.get_all_configurable_joints()

        for joint in configurable_joints:
            if joint.limit and not (0 <= joint.limit.upper and 0 >= joint.limit.lower):
                joint_values.append((joint.limit.upper + joint.limit.lower) / 2.0)
            else:
                joint_values.append(0)
            joint_names.append(joint.name)
            joint_types.append(joint.type)
        return Configuration(joint_values, joint_types, joint_names)

    def zero_configuration(self, group=None):
        # type: (Optional[str]) -> Configuration
        """Get the zero joint configuration for the specified planning group.

        If the joint value `0.0` is outside of joint limits ``(joint.limit.upper, joint.limit.lower)`` then
        ``(upper + lower) / 2`` is used as joint value.

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`compas_robots.Configuration`
            The zero configuration of a planning group.

        Notes
        -----
        The zero configuration is not necessarily the full configuration of the robot
        unless the planning group contains all the configurable joints of the robot.

        Planning functions that requires a start_state (`RobotCellState`) as input
        must contain a full configuration in `RobotCellState.robot_configuration`, consider
        using the `zero_full_configuration` method to get a full configuration.

        Examples
        --------
        >>> robot_cell, robot_cell_state = RobotCellLibrary.ur5()
        >>> robot_cell.zero_configuration('manipulator')
        Configuration((0.000, 0.000, 0.000, 0.000, 0.000, 0.000), (0, 0, 0, 0, 0, 0), \
            ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'))
        """
        joint_values = []
        joint_names = []
        joint_types = []

        group = group or self.main_group_name
        configurable_joints = self.get_configurable_joints(group)

        for joint in configurable_joints:
            if joint.limit and not (0 <= joint.limit.upper and 0 >= joint.limit.lower):
                joint_values.append((joint.limit.upper + joint.limit.lower) / 2.0)
            else:
                joint_values.append(0)
            joint_names.append(joint.name)
            joint_types.append(joint.type)
        return Configuration(joint_values, joint_types, joint_names)

    def random_configuration(self, group=None):
        # type: (Optional[str]) -> Configuration
        """Get a random configuration for the specified planning group.

        This is not a full configuration of the robot, if a full configuration is needed,
        consider merging this configuration with the `zero_full_configuration`.

        For example: `robot_cell.zero_full_configuration().merged(robot_cell.random_configuration(group))`

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :class:`compas_robots.Configuration`

        Notes
        -----
        No collision checking is involved, the configuration may be in collision.
        """
        group = group or self.main_group_name
        configurable_joints = self.get_configurable_joints(group)
        joint_values = []
        joint_types = []
        joint_names = []
        for joint in configurable_joints:
            if joint.limit:
                joint_values.append(joint.limit.lower + (joint.limit.upper - joint.limit.lower) * random.random())
            else:
                joint_values.append(0)
            joint_types.append(joint.type)
            joint_names.append(joint.name)

        return Configuration(joint_values, joint_types, joint_names)

    def get_group_configuration(self, group, full_configuration):
        # type: (str, Configuration) -> Configuration
        """Filter a full configuration and return only the joints of the specified group.

        Parameters
        ----------
        group : :obj:`str`
            The name of the planning group.
        full_configuration : :class:`compas_robots.Configuration`
            A full configuration (with all configurable joints of the robot).
            Note that this object is not modified.

        Returns
        -------
        :class:`compas_robots.Configuration`
            A new configuration object with only the joints of the specified group.
        """
        # Adds joint_names to the configuration
        full_configuration = self.fill_configuration_with_joint_names(full_configuration)

        joint_names = self.get_configurable_joint_names(group)
        joint_types = self.get_configurable_joint_types(group)
        # Extract joint values from full configuration
        joint_values = [full_configuration[name] for name in joint_names]

        return Configuration(joint_values, joint_types, joint_names)

    def fill_configuration_with_joint_names(self, configuration, group=None):
        # type: (Configuration, Optional[str]) -> Configuration
        """Create a new configuration object from the given configuration and to fill in the joint_types and joint_names if they are missing.

        -   If the supplied configuration has joint_names and joint_types,
            a new configuration object with the same values is returned.
        -   If the supplied configuration is a full configuration (has all configurable joints of the robot),
            the `group` input has no effect.
        -   If the supplied configuration is a group configuration (has only the joints of a specific group),
            the corresponding group name must be passed.

        Returned Configuration is a new object and contain `.joint_names` attribute.

        Parameters
        ----------
        configuration : :class:`compas_robots.Configuration`, optional
            The configuration to fill with joint names and types.
        group : :obj:`str`, optional
            The name of the planning group. Defaults to None.

        Returns
        -------
        ::class:`compas_robots.Configuration`
            The configuration with joint names and types.
        """

        # If the configuration is already filled with joint names and types, return it.
        if len(configuration.joint_values) == len(configuration.joint_types) == len(configuration.joint_names):
            return Configuration(configuration.joint_values, configuration.joint_types, configuration.joint_names)

        # If the configuration is a full configuration, return it with joint names and types.
        all_configurable_joints = self.get_all_configurable_joints()
        if len(configuration.joint_values) == len(all_configurable_joints):
            joint_types = [joint.type for joint in all_configurable_joints]
            joint_names = [joint.name for joint in all_configurable_joints]
            return Configuration(configuration.joint_values, joint_types, joint_names)

        assert group, "Please provide the group name, as the configuration is not a full configuration."
        configurable_joints = self.get_configurable_joints(group)
        if len(configuration.joint_values) != len(configurable_joints):
            raise ValueError(
                "The number of configurable joints ({}) in the group '{}' does not match the number of joint values ({}) in the configuration".format(
                    len(configurable_joints), group, len(configuration.joint_values)
                )
            )
        joint_types = [joint.type for joint in configurable_joints]
        joint_names = [joint.name for joint in configurable_joints]
        return Configuration(configuration.joint_values, joint_types, joint_names)

    # ----------------------------------------------------------
    # Retrieval Functions related to a specific robot_cell_state
    # ----------------------------------------------------------

    def get_attached_tool(self, robot_cell_state, group):
        # type: (RobotCellState, str) -> Optional[ToolModel]
        """Return the ToolModel of the tool attached to the group in the robot cell state.

        There can only be a maximum of one tool attached to a planning group.

        Parameters
        ----------
        robot_cell_state : :class:`~compas_fab.robots.RobotCellState`
            The state of the robot cell.
            The tool attachment information is stored in the tool_states attribute.
        group : str
            The name of the planning group to which the tool is attached.
            This is not optional because the robot cell and and the state do not have
            knowledge of the main group name.

        Returns
        -------
        :class:`compas_robots.ToolModel` | None
            The tool attached to the group in the robot cell state.
            None if no tool is attached.
        """
        tool_id = robot_cell_state.get_attached_tool_id(group)
        return self.tool_models.get(tool_id)

    def get_attached_workpieces(self, robot_cell_state, group):
        # type: (RobotCellState, str) -> List[RigidBody]
        """Returns the workpiece attached to the tool attached to the group in the robot cell state.

        There can be more than one workpiece attached to a tool.

        Returns
        -------
        :class:`compas_fab.robots.RigidBody` | None
            The workpiece attached to the tool attached to the group in the robot cell state.
            None if no workpiece is attached or no tool is attached.
        """
        bodies = []
        workpiece_id = robot_cell_state.get_attached_workpiece_ids(group)
        for id in workpiece_id:
            self.rigid_body_models[id]
        return bodies

    def get_attached_rigid_bodies(self, robot_cell_state, group):
        # type: (RobotCellState, str) -> List[Mesh]
        """Returns the rigid bodies attached to the links of the robot as AttachedCollisionMesh.

        This does not include the tool and the workpieces attached to the tools.
        Use `get_attached_tool` and `get_attached_workpieces` to get those.

        """
        bodies = []
        rigid_body_ids = robot_cell_state.get_attached_rigid_body_ids()
        for id in rigid_body_ids:
            bodies.append(self.rigid_body_models[id])
        return bodies

    # ----------------------------------------
    # Transformation functions
    # ----------------------------------------

    def t_pcf_tcf(self, robot_cell_state, tool_id):
        # type: (RobotCellState, str) -> Transformation
        """Returns the transformation from the PCF (Planner Coordinate Frame) to the TCF (Tool Coordinate Frame).

        Parameters
        ----------
        tool_id : str
            The id of a tool found in `self.robot_cell.tool_models`.
            The tool must be attached to the robot.

        Returns
        -------
        :class:`~compas.geometry.Transformation`
            Transformation from the tool's TCF to TBCF.
        """

        if tool_id not in self.tool_models:
            raise ValueError("Tool with id '{}' not found in robot cell.".format(tool_id))
        tool_model = self.tool_models[tool_id]
        tool_state = robot_cell_state.tool_states[tool_id]
        if not tool_state.attached_to_group:
            raise ValueError("Tool with id '{}' is not attached to the robot.".format(tool_id))

        # The following precomputed transformations are used to speed up batch frame conversions.
        # t_pcf_tbcf is Tool Attachment Frame, describing Tool Base Coordinate Frame (TBCF) relative to PCF Frame on the Robot (PCF)
        attachment_frame = tool_state.attachment_frame or Frame.worldXY()
        t_pcf_tbcf = Transformation.from_frame(attachment_frame)
        # t_tbcf_tcf is Tool Frame, a property of the tool model, describing Tool Coordinate Frame (TCF) relative to Tool Base Frame (TBCF)
        t_tbcf_tcf = Transformation.from_frame(tool_model.frame)

        t_pcf_tcf = t_pcf_tbcf * t_tbcf_tcf
        return t_pcf_tcf

    def t_pcf_ocf(self, robot_cell_state, workpiece_id):
        # type: (RobotCellState, str) -> Transformation
        """Returns the transformation from the PCF (Planner Coordinate Frame) to the OCF (Object Coordinate Frame).

        Parameters
        ----------
        workpiece_id : str
            The id of a workpiece found in `self.rigid_body_models`.
            The workpiece must be attached to a tool, and the tool must be attached to the robot.

        Returns
        -------
        :class:`~compas.geometry.Transformation`
            Transformation from the workpiece's OCF to PCF.
        """

        if workpiece_id not in self.rigid_body_models:
            raise ValueError("Workpiece with id '{}' not found in robot cell.".format(workpiece_id))
        workpiece_state = robot_cell_state.rigid_body_states[workpiece_id]
        if not workpiece_state.attached_to_tool:
            raise ValueError("Workpiece with id '{}' is not attached to any tool.".format(workpiece_id))
        tool_id = workpiece_state.attached_to_tool
        if tool_id not in self.tool_models:
            raise ValueError(
                "Workpiece is attached to a Tool with id '{}', but the tool is not found in robot cell.".format(tool_id)
            )
        tool_state = robot_cell_state.tool_states[tool_id]
        if not tool_state.attached_to_group:
            raise ValueError("Tool with id '{}' is not attached to the robot.".format(tool_id))

        workpiece_attachment_frame = workpiece_state.attachment_frame or Frame.worldXY()
        t_tcf_ocf = Transformation.from_frame(workpiece_attachment_frame)
        t_pcf_tcf = self.t_pcf_tcf(robot_cell_state, tool_id)

        t_pcf_ocf = t_pcf_tcf * t_tcf_ocf
        return t_pcf_ocf

    def from_tcf_to_pcf(self, robot_cell_state, tcf_frames, tool_id):
        # type: (RobotCellState, List[Frame], str) -> List[Frame]
        """Converts a frame describing the robot's tool coordinate frame (TCF) relative to WCF
        to a frame describing the planner coordinate frame (PCF), relative to WCF.
        The transformation goes through the tool's base frame, which differs from the
        PCF by the tool's current attachment_frame (in tool_state).

        This tool specified by the tool_id must be attached to the robot in the robot cell state.

        This function is typically used by the planner
        at the beginning of the inverse kinematics calculation
        to convert the frame of the robot's tool tip (tcf) to the frame of the robot's flange (tool0).

        Parameters
        ----------
        robot_cell_state : :class:`~compas_fab.robots.RobotCellState`
            The state of the robot cell.
        tcf_frames : list of :class:`~compas.geometry.Frame`
            Tool Coordinate Frames (TCF) relative to the World Coordinate Frame (WCF).
        tool_id : str
            The id of a tool found in `client.robot_cell.tool_models`.
            The tool must be attached to the robot.

        Returns
        -------
        :class:`~compas.geometry.Frame`
            Planner Coordinate Frames (PCF) (also T0CF) relative to the World Coordinate Frame (WCF).
        """
        self.assert_cell_state_match(robot_cell_state)

        t_pcf_tcf = self.t_pcf_tcf(robot_cell_state, tool_id)
        t_tcf_pcf = t_pcf_tcf.inverse()

        planner_coordinate_frames = []
        for tc_frame in tcf_frames:
            # Convert input to Transformation
            t_wcf_tcf = Transformation.from_frame(tc_frame)

            # Combined transformation gives the PCF relative to the world coordinate frame
            t_wcf_pcf = t_wcf_tcf * t_tcf_pcf
            planner_coordinate_frames.append(Frame.from_transformation(t_wcf_pcf))

        return planner_coordinate_frames

    def from_pcf_to_tcf(self, robot_cell_state, pcf_frames, tool_id):
        # type: (RobotCellState, List[Frame], str) -> List[Frame]
        """Converts a frame describing the planner coordinate frame (PCF) (also T0CF) relative to WCF
        to a frame describing the robot's tool coordinate frame (TCF) relative to WCF.
        The transformation goes through the tool's base frame, which differs from the
        PCF by the tool's current attachment_frame (in tool_state).

        This tool specified by the tool_id must be attached to the robot in the robot cell state.

        This is typically used by the planner
        at the end of the forward kinematics calculation
        to convert the frame of the robot's flange (tool0) to the frame of the robot's tool tip (tcf).

        Parameters
        ----------
        robot_cell_state : :class:`~compas_fab.robots.RobotCellState`
            The state of the robot cell.
        pcf_frames : list of :class:`~compas.geometry.Frame`
            Planner Coordinate Frames (PCF) (also T0CF) relative to the World Coordinate Frame (WCF).
        tool_id : str
            The id of a tool found in `client.robot_cell.tool_models`.
            The tool must be attached to the robot.

        Returns
        -------
        :class:`~compas.geometry.Frame`
            Tool Coordinate Frames (TCF) relative to the World Coordinate Frame (WCF).
        """
        self.assert_cell_state_match(robot_cell_state)

        t_pcf_tcf = self.t_pcf_tcf(robot_cell_state, tool_id)

        tool_coordinate_frames = []
        for pcf in pcf_frames:
            # Convert input to Transformation
            t_wcf_pcf = Transformation.from_frame(pcf)

            # Combined transformation gives TCF of the tool relative to the world coordinate frame
            t_wcf_tcf = t_wcf_pcf * t_pcf_tcf
            tool_coordinate_frames.append(Frame.from_transformation(t_wcf_tcf))

        return tool_coordinate_frames

    def from_ocf_to_pcf(self, robot_cell_state, ocf_frames, workpiece_id):
        # type: (RobotCellState, List[Frame], str) -> List[Frame]
        """Converts a frame describing the object coordinate frame (OCF) relative to WCF
        to a frame describing the planner coordinate frame (PCF) (also T0CF) relative to WCF.
        The transformation goes from the workpiece's base frame,
        through the workpiece's attachment frame (in workpiece_state), to the tool's TCF.

        This workpiece specified by the workpiece_id must be attached to a tool,
        and the tool must be attached to the robot in the robot cell state.

        This is typically used by the planner at the beginning of the inverse kinematics calculation to convert
        the frame of the object (ocf) to the frame of the robot's flange (tool0).

        Parameters
        ----------
        robot_cell_state : :class:`~compas_fab.robots.RobotCellState`
            The state of the robot cell.
        ocf_frames : list of :class:`~compas.geometry.Frame`
            Object Coordinate Frames (OCF) relative to the World Coordinate Frame (WCF).
        workpiece_id : str
            The id of a workpiece found in `client.robot_cell.rigid_body_models`.
            The workpiece must be attached to a tool, and the tool must be attached to the robot.

        Returns
        -------
        :class:`~compas.geometry.Frame`
            Planner Coordinate Frames (PCF) (also T0CF) relative to the World Coordinate Frame (WCF).

        Notes
        -----
        This function works correctly even when there are multiple workpieces attached to the robot.
        Simply pass the correct workpiece_id to the function.

        """
        self.assert_cell_state_match(robot_cell_state)

        t_pcf_ocf = self.t_pcf_ocf(robot_cell_state, workpiece_id)
        t_ocf_pcf = t_pcf_ocf.inverse()

        pcfs = []
        for ocf in ocf_frames:
            # Convert input to Transformation
            t_wcf_ocf = Transformation.from_frame(ocf)

            # Combined transformation gives the PCF relative to the world coordinate frame
            t_wcf_pcf = t_wcf_ocf * t_ocf_pcf
            pcfs.append(Frame.from_transformation(t_wcf_pcf))

        return pcfs

    def from_pcf_to_ocf(self, robot_cell_state, pcf_frames, workpiece_id):
        # type: (RobotCellState, List[Frame], str) -> List[Frame]
        """Converts a frame describing the planner coordinate frame (PCF) (also T0CF) relative to WCF
        to a frame describing the object coordinate frame (OCF) relative to WCF.

        The transformation goes from the tool's attachment frame (in ToolState),
        through the tool's `.frame` (in ToolModel),
        through the workpiece's attachment frame (in workpiece_state),
        arriving at the workpiece's base frame.

        This workpiece specified by the workpiece_id must be attached to a tool,
        and the tool must be attached to the robot in the robot cell state.

        Parameters
        ----------
        robot_cell_state : :class:`~compas_fab.robots.RobotCellState`
            The state of the robot cell.
        pcf_frames : list of :class:`~compas.geometry.Frame`
            Planner Coordinate Frames (PCF) (also T0CF) relative to the World Coordinate Frame (WCF).
        workpiece_id : str
            The id of a workpiece found in `client.robot_cell.rigid_body_models`.
            The workpiece must be attached to a tool, and the tool must be attached to the robot.

        Notes
        -----
        This function works correctly even when there are multiple workpieces attached to the robot.
        Simply pass the correct workpiece_id to the function.

        """
        self.assert_cell_state_match(robot_cell_state)

        t_pcf_ocf = self.t_pcf_ocf(robot_cell_state, workpiece_id)

        ocfs = []
        for pcf in pcf_frames:
            # Convert input to Transformation
            t_wcf_pcf = Transformation.from_frame(pcf)

            # Combined transformation gives OCF of the workpiece relative to the world coordinate frame
            t_wcf_ocf = t_wcf_pcf * t_pcf_ocf
            ocfs.append(Frame.from_transformation(t_wcf_ocf))

        return ocfs

    def target_frames_to_pcf(self, robot_cell_state, frame_or_frames, target_mode, group):
        # type: (RobotCellState, Frame | List[Frame], TargetMode | str, str) -> Frame | List[Frame]
        """Converts a Frame or a list of Frames to the PCF (Planner Coordinate Frame) relative to WCF.

        This function is intended to be used by the planner to convert target frames to PCF for planning.
        The transformation is equivalent to :meth:`from_tcf_to_pcf` when the target mode is `TargetMode.TOOL`,
        and equivalent to :meth:`from_ocf_to_pcf` when the target mode is `TargetMode.WORKPIECE`.
        If the target mode is `TargetMode.ROBOT`, the input frames are unchanged.

        Parameters
        ----------
        robot_cell_state : :class:`~compas_fab.robots.RobotCellState`
            The state of the robot cell.
        frame_or_frames : :class:`~compas.geometry.Frame` or list of :class:`~compas.geometry.Frame`
            The frame or frames to convert.
        target_mode : :class:`~compas_fab.robots.TargetMode` or str
            The target mode of the frame or frames.
        group : str
            The planning group to check. Must be specified.

        Returns
        -------
        :class:`~compas.geometry.Frame` or list of :class:`~compas.geometry.Frame`
            Planner Coordinate Frame (PCF) relative to the World Coordinate Frame (WCF).
            If the input is a single frame, the output will also be a single frame.
            If the input is a list of frames, the output will also be a list of frames.
        """
        self.assert_cell_state_match(robot_cell_state)
        assert group, "The group must be specified."

        # Pack a single frame into a list if it is not already a list
        input_is_not_list = not isinstance(frame_or_frames, list)
        frames = [frame_or_frames] if input_is_not_list else frame_or_frames

        pcf_frames = None
        if target_mode == TargetMode.TOOL:
            tool_id = robot_cell_state.get_attached_tool_id(group)
            pcf_frames = self.from_tcf_to_pcf(robot_cell_state, frames, tool_id)
        elif target_mode == TargetMode.WORKPIECE:
            workpiece_ids = robot_cell_state.get_attached_workpiece_ids(group)
            assert len(workpiece_ids) == 1, "Only one workpiece should be attached to the robot in group '{}'.".format(
                group
            )
            pcf_frames = self.from_ocf_to_pcf(robot_cell_state, frames, workpiece_ids[0])
        elif target_mode == TargetMode.ROBOT:
            pcf_frames = frames
        else:
            raise ValueError("Unsupported target mode: '{}'.".format(target_mode))

        return pcf_frames[0] if input_is_not_list else pcf_frames

    def pcf_to_target_frames(self, robot_cell_state, frame_or_frames, target_mode, group):
        # type: (RobotCellState, Frame | List[Frame], TargetMode | str, str) -> Frame | List[Frame]
        """Converts a (or a list of) Planner Coordinate Frame (PCF) to the target frame
        according to the target mode.

        For example, if the target mode is `TargetMode.TOOL`, the function will convert the PCF to the TCF.
        If the target mode is `TargetMode.WORKPIECE`, the function will convert the PCF to the workpiece's OCF.


        This function is the opposite of :meth:`target_frames_to_pcf`.

        Parameters
        ----------
        robot_cell_state : :class:`~compas_fab.robots.RobotCellState`
            The state of the robot cell.
        frame_or_frames : :class:`~compas.geometry.Frame` or list of :class:`~compas.geometry.Frame`
            The PCF frame or frames to convert.
        target_mode : :class:`~compas_fab.robots.TargetMode` or str
            The target mode of the frame or frames.
        group : str
            The planning group to check. Must be specified.

        Returns
        -------
        :class:`~compas.geometry.Frame` or list of :class:`~compas.geometry.Frame`
            Target Frame relative to the World Coordinate Frame (WCF).
            If the input is a single frame, the output will also be a single frame.
            If the input is a list of frames, the output will also be a list of frames.
        """
        self.assert_cell_state_match(robot_cell_state)
        assert group, "The group must be specified."

        # Pack a single frame into a list if it is not already a list
        input_is_not_list = not isinstance(frame_or_frames, list)
        frames = [frame_or_frames] if input_is_not_list else frame_or_frames

        target_frames = None

        if target_mode == TargetMode.TOOL:
            tool_id = robot_cell_state.get_attached_tool_id(group)
            target_frames = self.from_pcf_to_tcf(robot_cell_state, frames, tool_id)
        elif target_mode == TargetMode.WORKPIECE:
            workpiece_ids = robot_cell_state.get_attached_workpiece_ids(group)
            assert len(workpiece_ids) == 1, "Only one workpiece should be attached to the robot in group '{}'.".format(
                group
            )
            target_frames = self.from_pcf_to_ocf(robot_cell_state, frames, workpiece_ids[0])
        elif target_mode == TargetMode.ROBOT:
            target_frames = frames
        else:
            raise ValueError("Unsupported target mode: '{}'.".format(target_mode))

        return target_frames[0] if input_is_not_list else target_frames

    # ----------------------------------------
    # Debug Functions
    # ----------------------------------------

    def print_info(self):
        # type: () -> None
        """Print information about the robot."""
        print("The robot's name is '{}'.".format(self.robot_model.name))
        print("The robot's joints are:")
        for joint in self.robot_model.iter_joints():
            joint = joint  # type: Joint
            info = "\t* '{}' is of type '{}'".format(joint.name, list(Joint.SUPPORTED_TYPES)[joint.type])
            if joint.limit:
                info += " and has limits [{:.3f}, {:.3f}]".format(joint.limit.upper, joint.limit.lower)
            if joint.type == Joint.FIXED:
                info += " (Fixed)"
            elif joint.mimic:
                info += " (Mimics '{}')".format(joint.mimic.joint)
            else:
                info += " (Configurable)"
            print(info)

        print("The robot's links are:")
        print([link.name for link in self.robot_model.links])

        if not self.robot_semantics:
            print("No Semantics is not available. Please set the semantics to enable planning")
        else:
            print("The planning groups are:", self.group_names)
            print("The main planning group is '{}'.".format(self.main_group_name))
            print("The base link's name is '{}'".format(self.get_base_link_name()))
            print("The end-effector's link name is '{}'.".format(self.get_end_effector_link_name()))

        print("The following tools are present:")
        print([self.tool_models.keys()])

        print("The following rigid bodies are present:")
        print([self.rigid_body_models.keys()])
