from compas.data import Data
from compas.geometry import Frame, Transformation  # noqa F401
from compas.robots import Configuration  # noqa F401

try:
    from typing import Optional  # noqa F401
    from typing import Dict  # noqa F401
except ImportError:
    pass

__all__ = [
    "SceneState",
    "WorkpieceState",
    "ToolState",
    "RobotState",
]


class SceneState(Data):
    """Container for the states of all workpieces, tools and robot in a static scene.

    Current implementation supports multiple workpieces and tools, but only one robot.

    No more than one tool can be attached to the robot at any time. Similarly,
    no more than one workpiece can be attached to the tool at any time.
    It is not possible to attach a workpiece to the robot directly.
    Use a dummy tool (with identity transformation and no visual or collision meshes) if necessary.

    SceneState provides convinence functions such as :meth:`SceneState.get_attached_tool_id()`
    and :meth:`SceneState.get_attached_workpiece_id()` for identifying which tool and workpiece
    are currently attached to the robot.

    When constructing a SceneState, all the workpieces and tools id should be provided
    to the constructor. Default :class:`WorkpieceState`, :class:`ToolState` and
    :class:`RobotState` are automatically created and can be accessed by
    :meth:`SceneState.get_workpiece_state()`, :meth:`SceneState.get_tool_state()` and
    :meth:`SceneState.get_robot_state()`.


    Attributes
    ----------
    workpiece_ids : list of str
        The ids of the workpieces in the scene.
    tool_ids : list of str
        The ids of the tools in the scene.
    workpiece_states : `dict` of :class:`WorkpieceState`
        The states of the workpieces in the scene. The keys are the workpiece ids.
    tool_states : `dict` of :class:`ToolState`
        The states of the tools in the scene. The keys are the tool ids.
    robot_state : :class:`RobotState`
        The state of the robot in the scene.
    """

    def __init__(self, workpiece_ids=[], tool_ids=[]):
        super(SceneState, self).__init__()
        self.workpiece_ids = workpiece_ids  # type: list[str]
        self.tool_ids = tool_ids
        self.workpiece_states = {}  # type: dict[str, WorkpieceState]
        self.tool_states = {}  # type: dict[str, ToolState]
        self.robot_state = RobotState()  # type: RobotState

        # Create initial states for all workpieces and tools
        for workpiece_id in workpiece_ids:
            self.workpiece_states[workpiece_id] = WorkpieceState(workpiece_id)
        for tool_id in tool_ids:
            self.tool_states[tool_id] = ToolState(tool_id)
        self.workpiece_states["s"].attached_to_tool_grasp
        self.tool_states["s"].attached_to_robot_grasp

    @property
    def data(self):
        data = {}
        data["workpiece_ids"] = self.workpiece_ids
        data["tool_ids"] = self.tool_ids
        data["workpiece_states"] = self.workpiece_states
        data["tool_states"] = self.tool_states
        data["robot_state"] = self.robot_state
        return data

    @data.setter
    def data(self, data):
        self.workpiece_ids = data.get("workpiece_ids", self.workpiece_ids)
        self.tool_ids = data.get("tool_ids", self.tool_ids)
        self.workpiece_states = data.get("workpiece_states", self.workpiece_states)
        self.tool_states = data.get("tool_states", self.tool_states)
        self.robot_state = data.get("robot_state", self.robot_state)

    def get_robot_state(self):
        # type: () -> RobotState
        """Returns the state of the only robot in the scene."""
        return self.robot_state

    def get_tool_state(self, tool_id=None):
        # type: (str) -> ToolState
        """Returns the state of a tool by tool id.
        If there is only one tool in the scene, the tool id can be omitted.

        Parameters
        ----------
        tool_id : str, optional
            The id of the tool.
            If tool_id is `None` and there is only one tool in the scene, the tool id is inferred.
        """
        if tool_id is None:
            if len(self.tool_ids) > 1:
                raise ValueError("There is more than one tool in the scene. Please specify the tool id.")
            tool_id = self.tool_ids[0]
        return self.tool_states[tool_id]

    def get_workpiece_state(self, workpiece_id):
        # type: (str) -> WorkpieceState
        """Returns the state of a workpiece by workpiece id.

        Parameters
        ----------
        workpiece_id : str
            The id of the workpiece.
        """
        return self.workpiece_states[workpiece_id]

    def get_attached_tool_id(self):
        # type: () -> Optional[str]
        """Returns the id of the tool that is currently attached to the robot.
        This function assumes there is only one possible tool attached to the robot.
        If no tool is attached, `None` is returned.
        """
        for tool_id, tool_state in self.tool_states.items():
            if tool_state.attached_to_robot:
                return tool_id
        return None

    def get_attached_workpiece_id(self):
        # type: () -> Optional[str]
        """Returns the id of the workpiece that is currently attached to the robot.
        This function assumes there is only one possible workpiece attached to the robot.
        If no workpiece is attached, `None` is returned.
        """
        for workpiece_id, workpiece_state in self.workpiece_states.items():
            if workpiece_state.attached_to_tool_id:
                return workpiece_id
        return None


class WorkpieceState(Data):
    """Class for describing the state of a workpiece.

    WorkpieceState objects are typically created by the constructor of :class:`SceneState`.

    Attributes
    ----------
    workpiece_id : str
        Unique identifier of the workpiece used in Process.workpieces and SceneState.workpiece_states.
    frame : :class:`compas.geometry.Frame`
        The current location of the workpiece.
        (default: :class:`compas.geometry.Frame.worldXY`)
    attached_to_tool_id : str or None
        If the workpiece is attached to a tool, the id of the tool.
        If the workpiece is not attached to a tool, `None`. (default: `None`)
    attached_to_tool_grasp : :class:`compas.geometry.Transformation`
        If the workpiece is attached to a tool, the grasp frame of the workpiece.
        If not specified, defaults to the identity transformation.
        If the workpiece is not attached to a tool, `None`.
    is_hidden : bool
        If the workpiece is hidden, `True`. Else, `False`. (default: `False`)
        A hidden workpiece will not be included for collision detection of the scene.
    """

    def __init__(self, workpiece_id="undefined_workpiece"):
        super(WorkpieceState, self).__init__()
        self.workpiece_id = workpiece_id
        self.frame = Frame.worldXY()  # type: Frame
        self.attached_to_tool_id = None  # type: Optional[str]
        self.attached_to_tool_grasp = Transformation()  # type: Optional[Transformation]
        self.is_hidden = False  # type: bool

    @property
    def data(self):
        data = {}
        data["workpiece_id"] = self.workpiece_id
        data["frame"] = self.frame
        data["attached_to_tool_id"] = self.attached_to_tool_id
        data["attached_to_tool_grasp"] = self.attached_to_tool_grasp
        data["is_hidden"] = self.is_hidden
        return data

    @data.setter
    def data(self, data):
        self.workpiece_id = data.get("workpiece_id", self.workpiece_id)
        self.frame = data.get("frame", self.frame)
        self.attached_to_tool_id = data.get("attached_to_tool_id", self.attached_to_tool_id)
        self.attached_to_tool_grasp = data.get("attached_to_tool_grasp", self.attached_to_tool_grasp)
        self.is_hidden = data.get("is_hidden", self.is_hidden)


class ToolState(Data):
    """Class for describing the state of a tool.

    ToolState objects are typically created by the constructor of :class:`SceneState`.


    Attributes
    ----------
    tool_id : str
        Unique identifier of the tool used in Process.tools and SceneState.tool_states.
    frame : :class:`compas.geometry.Frame`
        The current location of the tool.
    attached_to_robot : bool
        If the tool is attached to a robot, `True`. Else, `False`.
    attached_to_robot_grasp : :class:`compas.geometry.Transformation`
        If the tool is attached to a robot, the base frame of the tool relative to the robot flange.
    configuration : :class:`compas.robots.Configuration`, optional
        If the tool is kinematic, the current configuration of the tool.
    """

    def __init__(self, tool_id="undefined_tool", configuration=None):
        super(ToolState, self).__init__()
        self.tool_id = tool_id
        self.frame = Frame.worldXY()  # type: Frame
        self.attached_to_robot = False  # type: bool
        self.attached_to_robot_grasp = None  # type: Optional[Transformation]
        self.configuration = configuration  # type: Optional[Configuration]

    @property
    def data(self):
        data = {}
        data["tool_id"] = self.tool_id
        data["frame"] = self.frame
        data["attached_to_robot"] = self.attached_to_robot
        data["attached_to_robot_grasp"] = self.attached_to_robot_grasp
        data["configuration"] = self.configuration
        return data

    @data.setter
    def data(self, data):
        self.tool_id = data.get("tool_id", self.tool_id)
        self.frame = data.get("frame", self.frame)
        self.attached_to_robot = data.get("attached_to_robot", self.attached_to_robot)
        self.attached_to_robot_grasp = data.get("attached_to_robot_grasp", self.attached_to_robot_grasp)
        self.configuration = data.get("configuration", self.configuration)


class RobotState(Data):
    """Class for describing the state of a robot.

    RobotState objects are typically created by the constructor of :class:`SceneState`.
    However it is possible to create a RobotState object manually. For example, when specifying
    the initial state of a robot in a planning process.


    Attributes
    ----------
    frame : :class:`compas.geometry.Frame`
        The current flange frame (robot target) of the robot.
    configuration : :class:`compas.robots.Configuration`
        The current configuration of the robot.
    """

    def __init__(self,  frame=None, configuration=None):
        super(RobotState, self).__init__()
        self.frame = frame  # type: Frame
        self.configuration = configuration  # type: Optional[Configuration]

    @property
    def data(self):
        data = {}
        data["frame"] = self.frame
        data["configuration"] = self.configuration
        return data

    @data.setter
    def data(self, data):
        self.frame = data.get("frame", self.frame)
        self.configuration = data.get("configuration", self.configuration)
