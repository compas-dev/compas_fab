import compas

from compas.data import Data

if not compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Dict  # noqa: F401
        from typing import List  # noqa: F401
        from typing import Optional  # noqa: F401
        from compas.datastructures import Mesh  # noqa: F401
        from compas.geometry import Frame  # noqa: F401
        from compas_robots import Configuration  # noqa: F401
        from compas_robots import RobotModel  # noqa: F401
        from compas_robots import ToolModel  # noqa: F401
        from compas.geometry import Transformation  # noqa: F401
        from compas_fab.robots import Robot  # noqa: F401


class RobotCell(Data):
    """Represents objects in a robot cell. This include a single robot, a list of tools, and rigid bodies.

    Tools include those that are attached to the robot, and those that are not attached but placed in the environment.

    Rigid bodies are objects can be used to represent:
    - Workpieces that are attached to the tip of a tool
    - Workpieces that are placed in the environment and are not attached
    - Robotic backpacks or other accessories that are attached to links of the robot
    - Static obstacles in the environment
    """

    def __init__(self, robot_model, tool_models={}, rigid_body_models={}, static_environment_models={}):
        super(RobotCell, self).__init__()
        self.robot_model = robot_model  # type: RobotModel
        self.tool_models = tool_models  # type: Dict[str, ToolModel]
        self.rigid_body_models = rigid_body_models  # type: Dict[str, RigidBody]

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
        return {
            "robot_model": self.robot_model,
            "tool_models": self.tool_models,
            "rigid_body_models": self.rigid_body_models,
        }

    def assert_cell_state_match(self, cell_state):
        # type: (RobotCellState) -> None
        """Assert that the number of tools and rigid bodies in the cell state match the number of tools and workpieces in the robot cell."""
        symmetric_difference = set(cell_state.tool_ids) ^ set(self.tool_ids)
        if symmetric_difference != set():
            raise ValueError(
                "The tools in the cell state do not match the tools in the robot cell. Mismatch: %s"
                % symmetric_difference
            )
        symmetric_difference = set(cell_state.rigid_body_ids) ^ set(self.rigid_body_ids)
        if symmetric_difference != set():
            raise ValueError(
                "The workpieces in the cell state do not match the workpieces in the robot cell. Mismatch: %s"
                % symmetric_difference
            )

    def get_attached_tool(self, robot_cell_state, group):
        # type: (RobotCellState, str) -> Optional[ToolModel]
        """Returns the tool attached to the group in the robot cell state."""
        tool_id = robot_cell_state.get_attached_tool_id(group)
        if tool_id:
            return self.tool_models[tool_id]


class RigidBody(Data):
    """Represents a rigid body."""

    def __init__(self, name, visual_meshes=[], collision_meshes=[]):
        super(RigidBody, self).__init__()
        self.name = name  # type: str
        self.visual_meshes = visual_meshes  # type: List[Mesh]
        self.collision_meshes = collision_meshes  # type: List[Mesh]

    @property
    def __data__(self):
        return {
            "name": self.name,
            "visual_meshes": self.visual_meshes,
            "collision_meshes": self.collision_meshes,
        }


class RobotCellState(Data):
    """Represents the state of a robot cell.

    This class should be used to represent the complete state of a robot cell,
    not a partial state. The list of tool_states and rigid_body_states should
    match the list of tools and workpieces in the RobotCell.

    The only optional attribute is the robot configuration,
    which is not known before the motions are planned.

    """

    def __init__(self, robot_flange_frame, robot_configuration=None, tool_states={}, rigid_body_states={}):
        super(RobotCellState, self).__init__()
        self.robot_flange_frame = robot_flange_frame  # type: Frame
        self.robot_configuration = robot_configuration  # type: Optional[Configuration]
        self.tool_states = tool_states  # type: Dict[str, ToolState]
        self.rigid_body_states = rigid_body_states  # type: Dict[str, RigidBodyState]

    @property
    def tool_ids(self):
        # type: () -> List[str]
        return self.tool_states.keys()

    @property
    def rigid_body_ids(self):
        # type: () -> List[str]
        return self.rigid_body_states.keys()

    @property
    def __data__(self):
        return {
            "robot_configuration": self.robot_configuration,
            "tool_states": self.tool_states,
            "rigid_body_states": self.rigid_body_states,
        }

    @classmethod
    def from_robot_configuration(cls, robot, configuration=None, group=None):
        # type: (Robot, Optional[Configuration], Optional[str]) -> RobotCellState
        """Creates a `RobotCellState` from a robot and a configuration.

        Parameters
        ----------
        robot : :class:`~compas_robots.Robot`
            The robot.
        configuration : :class:`~compas_fab.Configuration`, optional
            The configuration of the robot. If the configuration is not provided, the robot's zero configuration will be used.
        group : str, optional
            The planning group used for calculation.
        """

        configuration = configuration or robot.zero_configuration(group)
        flange_frame = robot.forward_kinematics(configuration, group)
        return cls(flange_frame, configuration)

    def get_attached_tool_id(self, group):
        # type: (str) -> Optional[str]
        """Returns the id of the tool attached to the planning group."""
        for tool_id, tool_state in self.tool_states.items():
            if tool_state.attached_to_group == group:
                return tool_id


class ToolState(Data):
    """Represents the state of a tool in a RobotCell.

    When representing a tool that is attached to a robot, the `attached_to_group`
    attributes should be set to the planning group name. Otherwise, it should be `None`.
    Note that the tool's base frame is attached without any offset to the end of
    that planning group.

    When representing a tool that is kinematic (a ToolModel with movable joints), the
    `configuration` attribute should be set. Otherwise, if left at `None`, the tool's
    configuration will be assumed to be at its zero configuration. Note that grasp is
    relative to the base of the tool, and thus is not affected by the configuration of the tool.


    Attributes
    ----------
    frame : :class:`compas.geometry.Frame`
        The base frame of the tool relative to the world coordinate frame.
    attached_to_group : :obj:`str`, optional
        The name of the robot planning group to which the tool is attached. Defaults to ``None``.
    configuration : :class:`compas_robots.Configuration`, optional
        The configuration of the tool if the tool is kinematic. Defaults to ``None``.
    is_hidden : :obj:`bool`, optional
        Whether the tool is hidden in the scene. Collision checking will be turned off
        for hidden objects. Defaults to ``False``.
    """

    def __init__(self, frame, attached_to_group=None, configuration=None, is_hidden=False):
        super(ToolState, self).__init__()
        self.frame = frame  # type: Frame
        self.attached_to_group = attached_to_group  # type: Optional[str]
        self.configuration = configuration  # type: Optional[Configuration]
        self.is_hidden = is_hidden  # type: bool

    @property
    def __data__(self):
        return {
            "frame": self.frame,
            "attached_to_group": self.attached_to_group,
            "configuration": self.configuration,
            "is_hidden": self.is_hidden,
        }


class RigidBodyState(Data):
    """Represents the state of a workpiece in a RobotCell.

    When representing a workpiece that is attached to a tool, the `attached_to_tool` attribute should be set.
    When representing a collision geometry (such as robotic backpacks) that is attached to a link of the robot,
    the `attached_to_link` attribute should be set. Otherwise, they should be `None`.

    When representing a workpiece that is currently not in the scene, the `is_hidden` attribute should be set to True.
    This will turn off collision checking for the workpiece.

    Attributes
    ----------
    frame : :class:`compas.geometry.Frame`
        The base frame of the workpiece relative to the world coordinate frame.
    attached_to_link : :obj:`str`, optional
        The name of the robot link to which the workpiece is attached. Defaults to ``None``.
    attached_to_tool : :obj:`str`, optional
        The id of the tool to which the workpiece is attached. Defaults to ``None``.
    grasp : :class:`compas.geometry.Frame` | :class:`compas.geometry.Transformation`, optional
        The grasp frame of the workpiece relative to (the base frame of) the attached link or
        (the tool tip frame of) the tool. Defaults to ``None``.
    is_hidden : :obj:`bool`, optional
        Whether the workpiece is hidden in the scene. Collision checking will be turned off
        for hidden objects. Defaults to ``False``.
    """

    def __init__(self, frame, attached_to_link=None, attached_to_tool=None, grasp=None, is_hidden=False):
        if attached_to_link and attached_to_tool:
            raise ValueError("A RigidBodyState cannot be attached to both a link and a tool.")
