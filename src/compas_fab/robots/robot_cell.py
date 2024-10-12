from compas import IPY
from compas.data import Data
from compas.geometry import Frame
from compas.geometry import Transformation
from compas_robots import ToolModel

from compas_fab.backends.exceptions import BackendTargetNotSupportedError
from compas_fab.robots import TargetMode
from compas_fab.robots import Robot

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Dict  # noqa: F401
        from typing import List  # noqa: F401
        from typing import Optional  # noqa: F401

        from compas.datastructures import Mesh  # noqa: F401
        from compas_robots import Configuration  # noqa: F401
        from compas_robots import RobotModel  # noqa: F401

__all__ = [
    "RigidBody",
    "RigidBodyState",
    "RobotCell",
    "RobotCellState",
    "ToolState",
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
            raise BackendTargetNotSupportedError("Unsupported target mode: '{}'.".format(target_mode))

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
            raise BackendTargetNotSupportedError("Unsupported target mode: '{}'.".format(target_mode))

        return target_frames[0] if input_is_not_list else target_frames


class RigidBody(Data):
    """Represents a rigid body."""

    def __init__(self, visual_meshes, collision_meshes):
        # type: (List[Mesh] | Mesh, List[Mesh] | Mesh) -> None
        """Represents a rigid body.

        A rigid body can have different visual and collision meshes.

        Notes
        -----
        The number of objects in the collision meshes does not have to be the same as the visual meshes.
        If the user wants to use the same mesh for both visualization and collision checking,
        place the meshes in visual_meshes and leave the collision_meshes empty.

        Parameters
        ----------
        visual_meshes : list of :class:`compas.datastructures.Mesh` | :class:`compas.datastructures.Mesh`
            The visual meshes of the rigid body used for visualization purpose.
            They can be more detailed for realistic visualization without affecting planning performance.
        collision_meshes : list of :class:`compas.datastructures.Mesh` | :class:`compas.datastructures.Mesh`
            The collision meshes of the rigid body used for collision checking.
            They should be less detailed (fewer polygons) for better planning performance.
            If `None`, or an empty list is passed, no collision checking will be performed for the rigid body.

        Attributes
        ----------
        visual_meshes : list of :class:`compas.datastructures.Mesh`
            A list of meshes for visualization purpose.
        collision_meshes : list of :class:`compas.datastructures.Mesh`
            A list of meshes for collision checking.

        Notes
        -----
        compas_fab do not support weight and inertia properties for rigid bodies.

        """
        # type: (str, List[Mesh], List[Mesh]) -> None
        super(RigidBody, self).__init__()

        # If None is provided, we change that to an empty list
        if not visual_meshes:
            # If no input, set it to an empty list
            self.visual_meshes = []
        elif not isinstance(visual_meshes, list):
            # Ensure that it is a list
            self.visual_meshes = [visual_meshes]
        else:
            self.visual_meshes = visual_meshes

        if not collision_meshes:
            # If no input, set it to an empty list
            self.collision_meshes = []
        elif not isinstance(collision_meshes, list):
            # Ensure that it is a list
            self.collision_meshes = [collision_meshes]
        else:
            self.collision_meshes = collision_meshes

    @property
    def __data__(self):
        return {
            "visual_meshes": self.visual_meshes,
            "collision_meshes": self.collision_meshes,
        }

    @classmethod
    def from_mesh(cls, mesh):
        # type: (Mesh) -> RigidBody
        """Creates a RigidBody from a single mesh.

        This function is a convenience function for creating a RigidBody from a single mesh.
        The mesh will be used for both visualization and collision checking.

        Parameters
        ----------
        mesh : :class:`compas.datastructures.Mesh`
            The mesh of the rigid body.

        Returns
        -------
        :class:`compas_fab.robots.RigidBody`
            The rigid body.

        Notes
        -----
        If the user would like to use different meshes for visualization and collision checking,
        consider using the constructor directly: `RigidBody(visual_meshes, collision_meshes)`.

        """
        return cls([mesh], [mesh])

    @classmethod
    def from_meshes(cls, meshes):
        # type: (List[Mesh]) -> RigidBody
        """Creates a RigidBody from a list of meshes.

        This function is a convenience function for creating a RigidBody from a list of meshes.
        The first mesh will be used for visualization and collision checking.
        The rest of the meshes will be used for visualization only.

        Parameters
        ----------
        meshes : list of :class:`compas.datastructures.Mesh`
            The meshes of the rigid body.

        Returns
        -------
        :class:`compas_fab.robots.RigidBody`
            The rigid body.

        Notes
        -----
        If the user would like to use different meshes for visualization and collision checking,
        consider using the constructor directly: `RigidBody(visual_meshes, collision_meshes)`.

        """
        return cls(meshes, meshes)


class RobotCellState(Data):
    """Represents the state of a robot cell.

    This class should be used to represent the complete state of a robot cell,
    not a partial state. The list of tool_states and rigid_body_states should
    match the list of tools and workpieces in the RobotCell.

    The only optional attribute is the robot configuration,
    which is not known before the motions are planned.

    """

    def __init__(self, robot_flange_frame, robot_configuration=None, tool_states=None, rigid_body_states=None):
        # type: (Frame, Optional[Configuration], Dict[str, ToolState], Dict[str, RigidBodyState]) -> None
        super(RobotCellState, self).__init__()
        self.robot_flange_frame = robot_flange_frame  # type: Frame
        self.robot_configuration = robot_configuration  # type: Optional[Configuration]
        self.tool_states = tool_states or {}  # type: Dict[str, ToolState]
        self.rigid_body_states = rigid_body_states or {}  # type: Dict[str, RigidBodyState]

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
            "robot_flange_frame": self.robot_flange_frame,
            "robot_configuration": self.robot_configuration,
            "tool_states": self.tool_states,
            "rigid_body_states": self.rigid_body_states,
        }

    @classmethod
    def from_robot_cell(cls, robot_cell, robot_configuration=None):
        # type: (RobotCell, Optional[Configuration]) -> RobotCellState
        """Creates a default `RobotCellState` from a `RobotCell`.

        This function ensures that all the tools and workpieces in the robot cell are represented in the robot cell state.
        This function should be called after the robot cell is created and all objects are added to it.

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
        robot_cell_state = cls.from_robot_configuration(robot_cell.robot, robot_configuration)
        for tool_id, tool_model in robot_cell.tool_models.items():
            tool_state = ToolState(Frame.worldXY(), None, None)
            if len(tool_model.get_configurable_joints()) > 0:
                tool_state.configuration = tool_model.zero_configuration()
            robot_cell_state.tool_states[tool_id] = tool_state
        for rigid_body_id, rigid_body_model in robot_cell.rigid_body_models.items():
            rigid_body_state = RigidBodyState(Frame.worldXY(), None, None, None)
            robot_cell_state.rigid_body_states[rigid_body_id] = rigid_body_state
        return robot_cell_state

    @classmethod
    def from_robot_configuration(cls, robot, configuration=None, group=None):
        # type: (Robot | RobotModel, Optional[Configuration], Optional[str]) -> RobotCellState
        """Creates a `RobotCellState` from a robot and a configuration.

        This should be used only for robot cells that contain only the robot.
        The robot_flange_frame will be calculated using the forward kinematics of the robot model.

        Parameters
        ----------
        robot : :class:`~compas_fab.robots.Robot` or :class:`~compas_robots.RobotModel`
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
            return None
        ids = []
        for rigid_body_id, rigid_body_state in self.rigid_body_states.items():
            if rigid_body_state.attached_to_tool == tool_id:
                ids.append(rigid_body_id)
        return ids

    def get_attached_rigid_body_ids(self):
        # type: (str) -> List[str]
        """Returns the ids of the rigid bodies attached to the robot

        This does not include the tools attached to the robot and the workpieces attached to the tools.

        Returns
        -------
        List[str]
            The ids of the rigid bodies attached to the robot.
        """
        ids = []
        for rigid_body_id, rigid_body_state in self.rigid_body_states.items():
            if rigid_body_state.attached_to_link:
                ids.append(rigid_body_id)

    def set_tool_attached_to_group(self, tool_id, group, attachment_frame=None, touch_links=None, detach_others=True):
        # type: (str, str, Optional[Frame], Optional[List[str]], Optional[bool]) -> None
        """Sets the tool attached to the planning group.

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
        detach_others : bool, optional
            Whether to detach all other tools from the group. Defaults to True.
        """
        if not attachment_frame:
            attachment_frame = Frame.worldXY()

        self.tool_states[tool_id].attached_to_group = group
        self.tool_states[tool_id].frame = None
        self.tool_states[tool_id].attachment_frame = attachment_frame
        self.tool_states[tool_id].touch_links = touch_links or []

        if detach_others:
            for id, tool_state in self.tool_states.items():
                if id != tool_id and tool_state.attached_to_group == group:
                    tool_state.attached_to_group = None

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
    frame : :class:`compas.geometry.Frame`
        The base frame of the tool relative to the world coordinate frame.
        If the tool is attached to a planning group, this frame can be set to None.'
        In that case, the planner or visualization tool will use the end frame of the planning group.
    attached_to_group : :obj:`str`, optional
        The name of the robot planning group to which the tool is attached. Defaults to ``None``.
    attachment_frame : :class:`compas.geometry.Frame`, optional
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
        # type: (Frame, Optional[str], Optional[Frame], Optional[List[str]], Optional[Configuration], Optional[bool]) -> None
        super(ToolState, self).__init__()
        self.frame = frame  # type: Frame
        self.attached_to_group = attached_to_group  # type: Optional[str]
        self.touch_links = touch_links or []  # type: List[str]
        self.attachment_frame = attachment_frame  # type: Optional[Frame]
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
    frame : :class:`compas.geometry.Frame`
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
