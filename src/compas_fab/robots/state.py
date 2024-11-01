from compas import IPY
from compas.data import Data
from compas.geometry import Frame
from compas.geometry import Transformation

from .targets import TargetMode

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:  # pragma: no cover
        from typing import Dict  # noqa: F401
        from typing import List  # noqa: F401
        from typing import Optional  # noqa: F401

        from compas_robots import Configuration  # noqa: F401

        from compas_fab.robots import RobotCell  # noqa: F401

__all__ = [
    "RigidBodyState",
    "RobotCellState",
    "ToolState",
]


class RobotCellState(Data):
    """Represents the state of a robot cell.

    This class should be used to represent the complete state of a robot cell,
    not a partial state. The list of tool_states and rigid_body_states should
    match the list of tools and workpieces in the RobotCell.

    The only optional attribute is the robot configuration,
    which is not known before the motions are planned.

    """

    def __init__(
        self,
        robot_base_frame=None,
        robot_configuration=None,
        tool_states=None,
        rigid_body_states=None,
    ):
        # type: (Optional[Frame], Optional[Frame], Optional[Configuration], Dict[str, ToolState], Dict[str, RigidBodyState]) -> None
        super(RobotCellState, self).__init__()
        self.robot_base_frame = robot_base_frame or Frame.worldXY()  # type: Frame
        self.robot_configuration = robot_configuration  # type: Optional[Configuration]
        self.tool_states = tool_states or {}  # type: Dict[str, ToolState]
        self.rigid_body_states = rigid_body_states or {}  # type: Dict[str, RigidBodyState]

    @property
    def tool_ids(self):
        # type: () -> List[str]
        return list(self.tool_states.keys())

    @property
    def rigid_body_ids(self):
        # type: () -> List[str]
        return list(self.rigid_body_states.keys())

    @property
    def __data__(self):
        return {
            "robot_base_frame": self.robot_base_frame,
            "robot_configuration": self.robot_configuration,
            "tool_states": self.tool_states,
            "rigid_body_states": self.rigid_body_states,
        }

    def __eq__(self, value):
        # type: (RobotCellState) -> bool
        if value is None or not isinstance(value, RobotCellState):
            return False
        if self.robot_base_frame != value.robot_base_frame:
            return False
        if self.robot_configuration:
            try:
                if not self.robot_configuration.close_to(value.robot_configuration):
                    return False
            except Exception:
                return False
        else:
            if self.robot_configuration != value.robot_configuration:
                return False
        if set(self.tool_states.keys()) != set(value.tool_states.keys()):
            return False
        for tool_id, tool_state in self.tool_states.items():
            if tool_state is None and value.tool_states[tool_id] is None:
                continue
            if tool_state != value.tool_states[tool_id]:
                return False
        if set(self.rigid_body_states.keys()) != set(value.rigid_body_states.keys()):
            return False
        for rigid_body_id, rigid_body_state in self.rigid_body_states.items():
            if rigid_body_state is None and value.rigid_body_states[rigid_body_id] is None:
                continue
            if rigid_body_state != value.rigid_body_states[rigid_body_id]:
                return False
        return True

    @classmethod
    def from_robot_cell(cls, robot_cell, robot_configuration=None):
        # type: (RobotCell, Optional[Configuration]) -> RobotCellState
        """Creates a default `RobotCellState` from a `RobotCell`.

        This function ensures that all the tools and workpieces in the robot cell are represented in the robot cell state.
        This function should be called after the robot cell is created and all objects are added to it.

        The robot's base frame will be assumed to be at worldXY frame.
        All tools will be assumed to be in their zero configuration and positioned at worldXY frame.
        All workpieces will be assumed to be in their base frame and not attached to any tool or link.
        All tools and workpieces are assumed to be visible in the scene (is_hidden=False).

        Parameters
        ----------
        robot_cell : :class:`~compas_fab.robots.RobotCell`
            The robot cell.
        robot_configuration : :class:`~compas_fab.Configuration`, optional
            The configuration of the robot. If the configuration is not provided, the robot's zero configuration will be used.
        """
        robot_configuration = robot_configuration or robot_cell.zero_full_configuration()
        base_frame = Frame.worldXY()

        tool_states = {}
        for tool_id, tool_model in robot_cell.tool_models.items():
            tool_state = ToolState(Frame.worldXY())
            if len(tool_model.get_configurable_joints()) > 0:
                tool_state.configuration = tool_model.zero_configuration()
            tool_states[tool_id] = tool_state

        rigid_body_states = {}
        for rigid_body_id, rigid_body_model in robot_cell.rigid_body_models.items():
            rigid_body_state = RigidBodyState(Frame.worldXY())
            rigid_body_states[rigid_body_id] = rigid_body_state

        return cls(base_frame, robot_configuration, tool_states, rigid_body_states)

    def get_attached_tool_id(self, group):
        # type: (str) -> Optional[str]
        """Returns the id of the tool attached to the planning group.

        There can only be a maximum of one tool attached to a planning group.

        Returns
        -------
        str | None
            The id of the tool attached to the planning group.
            None if no tool is attached.
        """
        for tool_id, tool_state in self.tool_states.items():
            if tool_state.attached_to_group == group:
                return tool_id

    def get_detached_tool_ids(self):
        # type: () -> List[str]
        """Returns the ids of the tools that are not attached to any planning group.

        Returns
        -------
        List[str]
            The ids of the tools that are not attached to any planning group.
        """
        return [tool_id for tool_id, tool_state in self.tool_states.items() if not tool_state.attached_to_group]

    def get_attached_workpiece_ids(self, group):
        # type: (str) -> Optional[str]
        """Returns the id of the workpiece attached to the tool attached to the planning group.

        Workpieces are rigid bodies that are attached to the tip of a tool.
        There can be more than one workpiece attached to a tool.

        Returns
        -------
        List[str]
            The ids of the workpieces attached to the tool attached to the planning group.

        """
        tool_id = self.get_attached_tool_id(group)
        if not tool_id:
            return []
        ids = []
        for rigid_body_id, rigid_body_state in self.rigid_body_states.items():
            if rigid_body_state.attached_to_tool == tool_id:
                ids.append(rigid_body_id)
        return ids

    def get_attached_rigid_body_ids(self):
        # type: () -> List[str]
        """Returns the ids of the rigid bodies attached to links of the robot and to tools.

        Returns
        -------
        List[str]
            The ids of the rigid bodies attached to the robot.
        """
        ids = []
        for rigid_body_id, rigid_body_state in self.rigid_body_states.items():
            if rigid_body_state.attached_to_link or rigid_body_state.attached_to_tool:
                ids.append(rigid_body_id)
        return ids

    def set_tool_attached_to_group(self, tool_id, group, attachment_frame=None, touch_links=None):
        # type: (str, str, Optional[Frame], Optional[List[str]]) -> None
        """Sets the tool attached to the planning group.

        Any other tools that is attached to the specified planning group will be detached.

        Notes
        -----
        There can only be a maximum of one tool attached to a planning group.

        Parameters
        ----------
        tool_id : str
            The id of the tool.
        group : str
            The name of the planning group to which the tool is attached.
        attachment_frame : :class:`compas.geometry.Frame`, optional
            The frame of the tool relative to the end frame of the planning group.
            Defaults to None, which means that the tool's frame coincides with the end frame of the planning group.
        touch_links : list of str, optional
            The names of the robot links that are allowed to collide with the tool.

        """
        if not attachment_frame:
            attachment_frame = Frame.worldXY()

        self.tool_states[tool_id].attached_to_group = group
        self.tool_states[tool_id].frame = None
        self.tool_states[tool_id].attachment_frame = attachment_frame
        self.tool_states[tool_id].touch_links = touch_links or []

        # Detach other tools that are attached to the same group
        for id, tool_state in self.tool_states.items():
            if id != tool_id and tool_state.attached_to_group == group:
                self.set_tool_detached(id)

    def set_tool_detached(self, tool_id, frame=None, touch_links=None):
        # type: (str, Optional[Frame], Optional[List[str]]) -> None
        """Sets the tool to be detached from the planning group.

        Parameters
        ----------
        tool_id : str
            The id of the tool.
        frame : :class:`compas.geometry.Frame`, optional
            The frame of the tool (relative to the world coordinate frame) after detaching.
            Defaults to None, which means that the tool's frame is not changed.
        touch_links : list of str, optional
            The names of the robot links that are allowed to collide with the tool.
            Defaults to None, which means that the tool's touch links are reset to an empty list.
            If this behavior is not desired, consider modifying the tool_states directly
            or set the touch_links set explicitly.
        """
        self.tool_states[tool_id].attached_to_group = None

        if frame:
            self.tool_states[tool_id].frame = frame
        else:
            # If frame is not specified, the frame of the tool is not changed after detaching
            pass

        if touch_links is None:
            self.tool_states[tool_id].touch_links = []
        else:
            self.tool_states[tool_id].touch_links = touch_links

    def set_rigid_body_attached_to_link(self, rigid_body_id, link_name, attachment_frame=None, touch_links=None):
        # type: (str, str, Optional[Frame | Transformation], Optional[List[str]]) -> None
        """Sets the rigid body attached to the link of the robot.

        Notes
        -----
        There can be more than one rigid body attached to a link.
        A RigidBody cannot be attached to both a link and a tool.

        Parameters
        ----------
        rigid_body_id : str
            The id of the rigid body.
        link_name : str
            The name of the link to which the rigid body is attached.
        attachment_frame : :class:`compas.geometry.Frame` or :class:`compas.geometry.Transformation`, optional
            The frame of the rigid body relative to the link.
            Defaults to None, which means that the rigid body is at the base of the link.
        touch_links : list of str, optional
            The names of the robot links that are allowed to collide with the rigid body.
        """
        if not attachment_frame:
            attachment_frame = Frame.worldXY()

        self.rigid_body_states[rigid_body_id].attached_to_link = link_name
        self.rigid_body_states[rigid_body_id].attached_to_tool = None
        self.rigid_body_states[rigid_body_id].frame = None
        self.rigid_body_states[rigid_body_id].attachment_frame = attachment_frame
        self.rigid_body_states[rigid_body_id].touch_links = touch_links or []

    def set_rigid_body_attached_to_tool(self, rigid_body_id, tool_id, attachment_frame=None):
        # type: (str, str, Optional[Frame]) -> None
        """Sets the rigid body attached to the tool.

        Notes
        -----
        There can be more than one rigid body attached to the tip of a tool.
        A RigidBody cannot be attached to both a link and a tool.

        Parameters
        ----------
        rigid_body_id : str
            The id of the rigid body.
        tool_id : str
            The id of the tool to which the rigid body is attached.
        attachment_frame : :class:`compas.geometry.Frame`, optional
            The attachment (grasp) frame of the rigid body relative to the tool.
            Defaults to None, which means that the rigid body is at the tip of the tool.
        """
        if not attachment_frame:
            attachment_frame = Frame.worldXY()

        self.rigid_body_states[rigid_body_id].attached_to_link = None
        self.rigid_body_states[rigid_body_id].attached_to_tool = tool_id
        self.rigid_body_states[rigid_body_id].frame = None
        self.rigid_body_states[rigid_body_id].attachment_frame = attachment_frame

    def assert_target_mode_match(self, target_mode, group):
        # type: (TargetMode | None, str) -> None
        """Check if the current tool and workpiece attachment state support the specified TargetMode.

        If the `target_mode` is None,
        such as the cases for ConfigurationTarget and ConstraintSetTarget,
        this function will not perform any checks.

        This check is performed automatically by planning functions, it can also be called manually
        by the user to ensure that the robot cell state is correctly set up.

        Checks are performed as follows:

        - If target mode is `TargetMode.TOOL`, the specified planning group must have a tool attached.
        - If target mode is `TargetMode.WORKPIECE`, the specified planning group must have one and only one workpiece attached.

        Parameters
        ----------
        target_mode : :class:`compas_fab.robots.TargetMode` or None
            The target or waypoints to check.
        group : str
            The planning group to check. Must be specified.

        Raises
        ------
        :class:`compas_fab.backends.TargetModeMismatchError`
            If the target mode is `TOOL` and no tool is attached to the robot in the specified group.
            If the target mode is `WORKPIECE` and no (or more than one) workpiece is attached to the specified group.

        ValueError
            If the planning group is not specified.

        """
        # Import here to avoid circular imports
        from compas_fab.backends.exceptions import TargetModeMismatchError

        if target_mode is None:
            return

        if group is None:
            raise ValueError("Planning group must be specified.")

        # Checks for Tool Mode
        tool_id = self.get_attached_tool_id(group)
        if target_mode == TargetMode.TOOL:

            if tool_id is None:
                raise TargetModeMismatchError(
                    "Target mode is 'TOOL', but no tool is attached to the robot in group '{}'.".format(group)
                )

        # Checks for Workpiece Mode
        workpiece_ids = self.get_attached_workpiece_ids(group)
        if target_mode == TargetMode.WORKPIECE:
            if not workpiece_ids:
                raise TargetModeMismatchError(
                    "Target mode is 'WORKPIECE', but no workpiece is attached to the robot in group '{}'.".format(group)
                )
            if len(workpiece_ids) > 1:
                raise TargetModeMismatchError(
                    "Target mode is 'WORKPIECE', but more than one workpiece is attached to the robot in group '{}'.".format(
                        group
                    )
                )


class ToolState(Data):
    """Represents the state of a tool in a RobotCell.

    When representing a tool that is attached to a robot, the `attached_to_group`
    attributes should be set to the planning group name and the `frame` attribute
    is set to 'None'.
    Note that the tool's base frame is attached without any offset to the end of
    that planning group, this behavior is fixed.

    When representing a tool that is kinematic (a ToolModel with movable joints), the
    `configuration` attribute should be set. Otherwise, if left at `None`, the tool's
    configuration will be assumed to be at its zero configuration.
    Note that the attachment location of workpieces (RigidBody) to the tool cannot
    and will not be changed by the tool's configuration.

    Attributes
    ----------
    frame : :class:`compas.geometry.Frame` | :class:`compas.geometry.Transformation`
        The base frame of the tool relative to the world coordinate frame.
        If the tool is attached to a planning group, this frame can be set to None.'
        In that case, the planner or visualization tool will use the end frame of the planning group.
    attached_to_group : :obj:`str`, optional
        The name of the robot planning group to which the tool is attached. Defaults to ``None``.
    attachment_frame : :class:`compas.geometry.Frame` | :class:`compas.geometry.Transformation`, optional
        The frame of the tool relative to the frame of the attached link. Defaults to ``None``.
    touch_links : :obj:`list` of :obj:`str`
        The names of the robot links that are allowed to collide with the tool.
    configuration : :class:`compas_robots.Configuration`, optional
        The configuration of the tool if the tool is kinematic. Defaults to ``None``.
    is_hidden : :obj:`bool`, optional
        Whether the tool is hidden in the scene. Collision checking will be turned off
        for hidden objects. Defaults to ``False``.
    """

    def __init__(
        self,
        frame,
        attached_to_group=None,
        touch_links=None,
        attachment_frame=None,
        configuration=None,
        is_hidden=False,
    ):
        # type: (Frame | Transformation, Optional[str], Optional[Frame| Transformation], Optional[List[str]], Optional[Configuration], Optional[bool]) -> None
        super(ToolState, self).__init__()
        self.frame = frame  # type: Frame
        # Convert frame to a Frame if it is a Transformation
        if isinstance(frame, Transformation):
            self.frame = Frame.from_transformation(frame)
        self.attached_to_group = attached_to_group  # type: Optional[str]
        self.touch_links = touch_links or []  # type: List[str]
        self.attachment_frame = attachment_frame  # type: Optional[Frame]
        # Convert frame to a Frame if it is a Transformation
        if isinstance(attachment_frame, Transformation):
            self.attachment_frame = Frame.from_transformation(attachment_frame)
        self.configuration = configuration  # type: Optional[Configuration]
        self.is_hidden = is_hidden  # type: bool

    @property
    def __data__(self):
        return {
            "frame": self.frame,
            "attached_to_group": self.attached_to_group,
            "touch_links": self.touch_links,
            "attachment_frame": self.attachment_frame,
            "configuration": self.configuration,
            "is_hidden": self.is_hidden,
        }

    def __eq__(self, value):
        # type: (ToolState) -> bool
        if value is None:
            return False
        # In order to allow duck typing we do not assert value.__class__ to be equal to self.__class__
        # Therefore, we wrap the tests in try block, if any of the attributes cannot be compared,
        # it will raise exception and return False
        try:
            if self.frame != value.frame:
                return False
            if self.attached_to_group != value.attached_to_group:
                return False
            if set(self.touch_links) != set(value.touch_links):
                return False
            if self.attachment_frame != value.attachment_frame:
                return False
            if self.configuration:
                try:
                    if not self.configuration.close_to(value.configuration):
                        return False
                except Exception:
                    return False
            else:
                if self.configuration != value.configuration:
                    return False
            if self.is_hidden != value.is_hidden:
                return False
        except Exception:
            return False
        return True


class RigidBodyState(Data):
    """Represents the state of a workpiece in a RobotCell.

    Rigid bodies can be used to represent different types of objects in the robot cell:

    - Workpieces that are attached to the tip of a tool
    - Robotic backpacks or other accessories that are attached to links of the robot
    - Workpieces or Static obstacles in the environment
    - All of the above objects but are currently hidden

    When representing a workpiece that is attached to a tool, the `attached_to_tool` attribute should be
    set to the name of the tool. The `attachment_frame` attribute should be set to the grasp frame,
    which represents the position of the workpiece relative to the tool coordinate frame (TCF).

    When representing a collision geometry (such as robotic backpacks) that is attached to a link of the robot,
    the `attached_to_link` attribute should be set. The `attachment_frame` attribute should be set to the relative position
    of the rigid body to the base frame of the link.
    The `touch_links` attribute should be set to the names of the robot links that are allowed
    to collide with the object.

    In either one of the two attached cases, the `frame` attribute should be set to 'None' as the actual frame
    is determined automatically by the position of the attached link or tool.
    Even if the attribute is not set to None, it will be disregarded.

    When representing a stationary object in the environment, such as stationary workpiece or obstacles,
    both `attached_to_` attributes should be set to `None`. The `frame` attribute should be set to the base frame
    of the object relative to the world coordinate frame.
    If the stationary object touches the robot, the `touch_links` attribute should be set
    to the names of the robot links that are allowed to collide with the object.

    When representing a workpiece that is currently not in the scene, the `is_hidden` attribute should be set to True.
    This will hide the object from collision checking and visualization.

    Attributes
    ----------
    frame : :class:`compas.geometry.Frame`
        The base frame of the rigid body relative to the world coordinate frame.
    attached_to_link : :obj:`str` | None
        The name of the robot link to which the rigid body is attached.
       ``None`` if not attached to a link.
    attached_to_tool : :obj:`str` | None
        The id of the tool to which the rigid body is attached.
        ``None`` if not attached to a tool.
    touch_links : :obj:`list` of :obj:`str`
        The names of the robot links that are allowed to collide with the rigid body.
        ``[]`` if the rigid body is not allowed to collide with any robot links.
    touch_bodies : :obj:`list` of :obj:`str`
        The names of other rigid bodies (including tools) that are allowed to collide with the rigid body.
        ``[]`` if the rigid body is not allowed to collide with any other rigid bodies.
    attachment_frame : :class:`compas.geometry.Frame` | :class:`compas.geometry.Transformation`, optional
        The attachment (grasp) frame of the rigid body relative to (the base frame of) the attached link or
        (the tool tip frame of) the tool.
        ``None`` if the rigid body is not attached to a link or a tool.
    is_hidden : :obj:`bool`
        Whether the rigid body is hidden in the scene. Collision checking will be turned off
        for hidden objects.
        Defaults to ``False``.

    Parameters
    ----------
    frame : :class:`compas.geometry.Frame` | :class:`compas.geometry.Transformation`
        The base frame of the rigid body relative to the world coordinate frame.
    attached_to_link : :obj:`str` , optional
        The name of the robot link to which the rigid body is attached.
        Defaults to ``None``, meaning that the rigid body is not attached to a link.
    attached_to_tool : :obj:`str` , optional
        The id of the tool to which the rigid body is attached.
        Defaults to ``None``, meaning that the rigid body is not attached to a tool.
    touch_links : :obj:`list` of :obj:`str`, optional
        The names of the robot links that are allowed to collide with the rigid body.
        Defaults to ``[]``, meaning that the rigid body is not allowed to collide with any robot links.
    touch_bodies : :obj:`list` of :obj:`str`, optional
        The names of other rigid bodies (including tools) that are allowed to collide with the rigid body.
        Defaults to ``[]``, meaning that the rigid body is not allowed to collide with any other rigid bodies.
    attachment_frame : :class:`compas.geometry.Frame` | :class:`compas.geometry.Transformation`, optional
        The attachment (grasp) frame of the rigid body relative to (the base frame of) the attached link or
        (the tool tip frame of) the tool.
        Defaults to ``None``, meaning that the rigid body is not attached to a link or a tool.
    is_hidden : :obj:`bool`, optional
        Whether the rigid body is hidden in the scene. Collision checking will be turned off
        for hidden objects.
        Defaults to ``False``.

    """

    def __init__(
        self,
        frame,
        attached_to_link=None,
        attached_to_tool=None,
        touch_links=None,
        touch_bodies=None,
        attachment_frame=None,
        is_hidden=False,
    ):
        # type: (Frame, Optional[str], Optional[str], Optional[List[str]], Optional[List[str]], Optional[Frame], Optional[bool]) -> None
        super(RigidBodyState, self).__init__()
        self.frame = frame  # type: Frame
        # Convert frame to a Frame if it is a Transformation
        if isinstance(frame, Transformation):
            self.frame = Frame.from_transformation(frame)
        self.attached_to_link = attached_to_link  # type: Optional[str]
        self.attached_to_tool = attached_to_tool  # type: Optional[str]
        self.touch_links = touch_links or []  # type: List[str]
        self.touch_bodies = touch_bodies or []
        if attached_to_link and attached_to_tool:
            raise ValueError("A RigidBodyState cannot be attached to both a link and a tool.")
        self.attachment_frame = attachment_frame  # type: Optional[Frame]
        # Convert attachment_frame to a Frame if it is a Transformation
        if isinstance(attachment_frame, Transformation):
            self.attachment_frame = Frame.from_transformation(attachment_frame)
        self.is_hidden = is_hidden  # type: bool

    @property
    def __data__(self):
        return {
            "frame": self.frame,
            "attached_to_link": self.attached_to_link,
            "attached_to_tool": self.attached_to_tool,
            "touch_links": self.touch_links,
            "touch_bodies": self.touch_bodies,
            "attachment_frame": self.attachment_frame,
            "is_hidden": self.is_hidden,
        }

    def __eq__(self, value):
        # type: (RigidBodyState) -> bool
        if value is None:
            return False
        # Wrap the tests in try block, if any of the attributes cannot be compared,
        # it will raise exception and return False
        try:
            if self.frame != value.frame:
                return False
            if self.attached_to_link != value.attached_to_link:
                return False
            if self.attached_to_tool != value.attached_to_tool:
                return False
            if set(self.touch_links) != set(value.touch_links):
                return False
            if set(self.touch_bodies) != set(value.touch_bodies):
                return False
            if self.attachment_frame != value.attachment_frame:
                return False
            if self.is_hidden != value.is_hidden:
                return False
        except Exception:
            return False
        return True
