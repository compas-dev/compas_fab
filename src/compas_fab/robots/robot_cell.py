from compas import IPY
from compas.data import Data
from compas.geometry import Frame
from compas.geometry import Transformation
from compas_robots import ToolModel

from compas_fab.robots import Robot

from .rigid_body import RigidBody
from .targets import TargetMode

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Dict  # noqa: F401
        from typing import List  # noqa: F401
        from typing import Optional  # noqa: F401

        from compas.datastructures import Mesh  # noqa: F401

        from .state import RobotCellState  # noqa: F401
__all__ = [
    "RobotCell",
]


class RobotCell(Data):
    """Represents objects in a robot cell. This include a single robot, a list of tools, and rigid bodies.

    Tools include those that are attached to the robot, and those that are not attached but placed in the environment.

    Notes regarding scaling: All models in the RobotCell should be in meters. If user models are in millimeters,
    they should be scaled to meters before being added to the RobotCell.

    Rigid bodies are objects can be used to represent:
    - Workpieces that are attached to the tip of a tool
    - Workpieces that are placed in the environment and are not attached
    - Robotic backpacks or other accessories that are attached to links of the robot
    - Static obstacles in the environment


    Attributes
    ----------
    robot : :class:`~compas_fab.robots.Robot`
        The robot in the robot cell.
        The robot's semantics is required.
    tool_models : dict of str and :class:`~compas_robots.ToolModel`
        The tools in the robot cell.
        The key is the unique identifier for the tool.
    rigid_body_models : dict of str and :class:`~compas_fab.robots.RigidBody`
        The rigid bodies in the robot cell.
        The key is the unique identifier for the rigid body.
    """

    def __init__(self, robot=None, tool_models=None, rigid_body_models=None):
        super(RobotCell, self).__init__()
        self.robot = robot  # type: Robot
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
        tool_models = {id: tool.__data__ for id, tool in self.tool_models.items()}
        rigid_body_models = {id: rigid_body.__data__ for id, rigid_body in self.rigid_body_models.items()}
        return {
            "robot": self.robot.__data__,
            "tool_models": tool_models,
            "rigid_body_models": rigid_body_models,
        }

    # NOTE: The following code is what would be more elegant, but it is not working because of the ProxyObject deepcopy failure problem.
    #       The __from_data__ method was not even required in the original code.

    # @property
    # def __data__(self):
    #     return {
    #         "robot": self.robot,
    #         "tool_models": self.tool_models,
    #         "rigid_body_models": self.rigid_body_models,
    #     }

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
        robot = Robot.__from_data__(data["robot"])
        tool_models = {id: ToolModel.__from_data__(tool_data) for id, tool_data in data["tool_models"].items()}
        rigid_body_models = {
            id: RigidBody.__from_data__(rigid_body_data) for id, rigid_body_data in data["rigid_body_models"].items()
        }
        return cls(robot, tool_models, rigid_body_models)

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
