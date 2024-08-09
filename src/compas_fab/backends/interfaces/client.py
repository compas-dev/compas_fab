from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import compas

from compas.geometry import Frame
from compas.geometry import Transformation

from compas_fab.backends.exceptions import BackendFeatureNotSupportedError

if compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from compas_fab.robots import Robot
        from compas_fab.robots import RobotCell
        from compas_fab.robots import RobotCellState
        from typing import List
        from compas.geometry import Frame


class ClientInterface(object):
    """Interface for all backend clients.  Forwards all planning services and
    planning scene management to the planner.

    Attributes
    ----------
    robot : :class:`compas_fab.robots.Robot`, read-only
        The robot instance associated with the client.
    """

    def __init__(self):
        self._robot = None  # type: Robot

    @property
    def robot(self):
        # type: () -> Robot
        return self._robot


class PlannerInterface(object):
    """Interface for all backend planners.
    Planner is connected to a ClientInterface when initiated, and can access
    the client's robot instance.
    It is responsible for all planning services for the robot.

    Parameters
    ----------
    client : :class:`compas_fab.backends.interfaces.ClientInterface`, optional
        The client instance associated with the planner.
        Typically this is set by the backend client during initialization,
        using `planner = PlannerInterface(client)`.
        Consult the chosen backend client for how to set this attribute.

    Attributes
    ----------
    client : :class:`compas_fab.backends.interfaces.ClientInterface`, read-only
        The client instance associated with the planner.
        It cannot be changed after initialization.

    robot_cell : :class:`compas_fab.robots.RobotCell`, read-only
        The robot cell instance previously passed to the client.

    robot_cell_state : :class:`compas_fab.robots.RobotCellState`, read-only
        The last robot cell state instance passed to the client.


    """

    def __init__(self, client=None):
        if client:
            self._client = client
        self._robot_cell = None  # type: RobotCell
        self._robot_cell_state = None  # type: RobotCellState
        super(PlannerInterface, self).__init__()

    @property
    def client(self):
        # type: () -> ClientInterface
        return self._client

    @property
    def robot_cell(self):
        # type: () -> RobotCell
        return self._robot_cell

    @property
    def robot_cell_state(self):
        # type: () -> RobotCellState
        return self._robot_cell_state

    # ==========================================================================
    # collision objects robot cell and cell state
    # ==========================================================================

    def set_robot_cell(self, *args, **kwargs):
        """Pass the models in the robot cell to the planning client.

        The client keeps the robot cell models in memory and uses them for planning.
        Calling this method will override the previous robot cell in the client.
        It should be called only if the robot cell models have changed.

        Raises
        ------
        BackendFeatureNotSupportedError
            Planner does not have this feature.
        """
        raise BackendFeatureNotSupportedError("Assigned planner does not have this feature.")

    def set_robot_cell_state(self, *args, **kwargs):
        """Set the robot cell state to the client.

        The client requires a robot cell state at the beginning of each planning request.
        This cell state must correspond to the robot cell set earlier by :meth:`set_robot_cell`.

        This function is called automatically by planning functions that takes a start_state as input.
        Therefore it is typically not necessary for the user to call this function,
        except when used to trigger visualization using native backend canvas.

        Raises
        ------
        BackendFeatureNotSupportedError
            Planner does not have this feature.
        """
        raise BackendFeatureNotSupportedError("Assigned planner does not have this feature.")

    # ==========================================================================
    # planning services
    # ==========================================================================

    def inverse_kinematics(self, *args, **kwargs):
        """Default method for planner.

        Raises
        ------
        BackendFeatureNotSupportedError
            Planner does not have this feature.
        """
        raise BackendFeatureNotSupportedError("Assigned planner does not have this feature.")

    def forward_kinematics(self, *args, **kwargs):
        """Default method for planner.

        Raises
        ------
        BackendFeatureNotSupportedError
            Planner does not have this feature.
        """
        raise BackendFeatureNotSupportedError("Assigned planner does not have this feature.")

    def plan_motion(self, *args, **kwargs):
        """Default method for planner.

        Raises
        ------
        BackendFeatureNotSupportedError
            Planner does not have this feature.
        """
        raise BackendFeatureNotSupportedError("Assigned planner does not have this feature.")

    def plan_cartesian_motion(self, *args, **kwargs):
        """Default method for planner.

        Raises
        ------
        BackendFeatureNotSupportedError
            Planner does not have this feature.
        """
        raise BackendFeatureNotSupportedError("Assigned planner does not have this feature.")

    # ==========================================================================
    # collision objects and planning scene
    # ==========================================================================

    def get_planning_scene(self, *args, **kwargs):
        """Default method for planner.

        Raises
        ------
        BackendFeatureNotSupportedError
            Planner does not have this feature.
        """
        raise BackendFeatureNotSupportedError("Assigned planner does not have this feature.")

    def reset_planning_scene(self, *args, **kwargs):
        """Default method for planner.

        Raises
        ------
        BackendFeatureNotSupportedError
            Planner does not have this feature.
        """
        raise BackendFeatureNotSupportedError("Assigned planner does not have this feature.")

    def add_collision_mesh(self, *args, **kwargs):
        """Default method for planner.

        Raises
        ------
        BackendFeatureNotSupportedError
            Planner does not have this feature.
        """
        raise BackendFeatureNotSupportedError("Assigned planner does not have this feature.")

    def remove_collision_mesh(self, *args, **kwargs):
        """Default method for planner.

        Raises
        ------
        BackendFeatureNotSupportedError
            Planner does not have this feature.
        """
        raise BackendFeatureNotSupportedError("Assigned planner does not have this feature.")

    def append_collision_mesh(self, *args, **kwargs):
        """Default method for planner.

        Raises
        ------
        BackendFeatureNotSupportedError
            Planner does not have this feature.
        """
        raise BackendFeatureNotSupportedError("Assigned planner does not have this feature.")

    def add_attached_collision_mesh(self, *args, **kwargs):
        """Default method for planner.

        Raises
        ------
        BackendFeatureNotSupportedError
            Planner does not have this feature.
        """
        raise BackendFeatureNotSupportedError("Assigned planner does not have this feature.")

    def remove_attached_collision_mesh(self, *args, **kwargs):
        """Default method for planner.

        Raises
        ------
        BackendFeatureNotSupportedError
            Planner does not have this feature.
        """
        raise BackendFeatureNotSupportedError("Assigned planner does not have this feature.")

    # ==========================================================================
    # Tool and Workpiece Functions
    # ==========================================================================

    def from_tcf_to_pcf(self, tcf_frames, tool_id):
        # type: (List[Frame], str) -> List[Frame]
        """Converts a frame describing the robot's tool coordinate frame (TCF) relative to WCF
        to a frame describing the planner coordinate frame (PCF), relative to WCF.
        The transformation goes through the tool's base frame, which differs from the
        PCF by the tool's current attachment_frame (in tool_state).

        This function is restricted to be used only with tools that are currently
        attached to the robot. Before calling this function, make sure the last call to
        set_robot_cell_state() has the correct tool attachment information.

        This is typically used at the beginning of the inverse kinematics calculation to convert
        the frame of the robot's tool tip (tcf) to the frame of the robot's flange (tool0).

        The tool_id must correspond to a valid tool in `planner.robot_cell.tool_models`.

        Parameters
        ----------
        tcf_frames : list of :class:`~compas.geometry.Frame`
            Tool Coordinate Frames (TCF) relative to the World Coordinate Frame (WCF).
        tool_id : str
            The id of the tool attached to the robot.

        Returns
        -------
        :class:`~compas.geometry.Frame`
            Planner Coordinate Frames (PCF) (also T0CF) relative to the World Coordinate Frame (WCF).
        """
        if tool_id not in self.robot_cell.tool_models:
            raise ValueError("Tool with id '{}' not found in robot cell.".format(tool_id))
        tool_model = self.robot_cell.tool_models[tool_id]
        tool_state = self.robot_cell_state.tool_states[tool_id]
        if not tool_state.attached_to_group:
            raise ValueError("Tool with id '{}' is not attached to the robot.".format(tool_id))

        # The following precomputed transformations are used to speed up batch frame conversions.
        # t_pcf_tbcf is Tool Attachment Frame, describing Tool Base Coordinate Frame (TBCF) relative to PCF Frame on the Robot (PCF)
        attachment_frame = tool_state.attachment_frame or Frame.worldXY()
        t_pcf_tbcf = Transformation.from_frame(attachment_frame)
        t_tbcf_pcf = t_pcf_tbcf.inverse()
        # t_tbcf_tcf is Tool Frame, a property of the tool model, describing Tool Coordinate Frame (TCF) relative to Tool Base Frame (TBCF)
        t_tbcf_tcf = Transformation.from_frame(tool_model.frame)
        t_tcf_tbcf = t_tbcf_tcf.inverse()

        # Note: The PCF in the world coordinate frame is given by t_wcf_pcf
        # t_wcf_t0cf is Tool0 Frame of the robot obtained from FK, describing Tool Base Frame (T0CF) relative to World Coordinate Frame (WCF)
        planner_coordinate_frames = []
        for tc_frame in tcf_frames:
            # Convert input to Transformation
            t_wcf_tcf = Transformation.from_frame(tc_frame)

            # Combined transformation gives the position of the tool in the world coordinate frame
            t_wcf_pcf = t_wcf_tcf * t_tcf_tbcf * t_tbcf_pcf
            planner_coordinate_frames.append(Frame.from_transformation(t_wcf_pcf))

        return planner_coordinate_frames

    def from_pcf_to_tcf(self, pcf_frames, tool_id):
        # type: (List[Frame], str) -> List[Frame]
        """Converts a frame describing the planner coordinate frame (PCF) (also T0CF) relative to WCF
        to a frame describing the robot's tool coordinate frame (TCF) relative to WCF.
        The transformation goes through the tool's base frame, which differs from the
        PCF by the tool's current attachment_frame (in tool_state).

        This function is restricted to be used only with tools that are currently
        attached to the robot. Before calling this function, make sure the last call to
        set_robot_cell_state() has the correct tool attachment information.

        This is typically used at the end of the forward kinematics calculation to convert
        the frame of the robot's flange (tool0) to the frame of the robot's tool tip (tcf).

        The tool_id must correspond to a valid tool in `planner.robot_cell.tool_models`.

        Parameters
        ----------
        pcf_frames : list of :class:`~compas.geometry.Frame`
            Planner Coordinate Frames (PCF) (also T0CF) relative to the World Coordinate Frame (WCF).
        tool_id : str
            The id of the tool attached to the robot.

        Returns
        -------
        :class:`~compas.geometry.Frame`
            Tool Coordinate Frames (TCF) relative to the World Coordinate Frame (WCF).
        """
        if tool_id not in self.robot_cell.tool_models:
            raise ValueError("Tool with id '{}' not found in robot cell.".format(tool_id))
        tool_model = self.robot_cell.tool_models[tool_id]
        tool_state = self.robot_cell_state.tool_states[tool_id]
        if not tool_state.attached_to_group:
            raise ValueError("Tool with id '{}' is not attached to the robot.".format(tool_id))

        # The following precomputed transformations are used to speed up batch frame conversions.
        # t_pcf_tbcf is Tool Attachment Frame, describing Tool Base Coordinate Frame (TBCF) relative to PCF Frame on the Robot (PCF)
        attachment_frame = tool_state.attachment_frame or Frame.worldXY()
        t_pcf_tbcf = Transformation.from_frame(attachment_frame)
        # t_tbcf_tcf is Tool Frame, a property of the tool model, describing Tool Coordinate Frame (TCF) relative to Tool Base Frame (TBCF)
        t_tbcf_tcf = Transformation.from_frame(tool_model.frame)

        # Note: The PCF in the world coordinate frame is given by t_wcf_pcf
        # t_wcf_t0cf is Tool0 Frame of the robot obtained from FK, describing Tool Base Frame (T0CF) relative to World Coordinate Frame (WCF)
        tool_coordinate_frames = []
        for pcf in pcf_frames:
            # Convert input to Transformation
            t_wcf_pcf = Transformation.from_frame(pcf)

            # Combined transformation gives the position of the tool in the world coordinate frame
            t_wcf_tcf = t_wcf_pcf * t_pcf_tbcf * t_tbcf_tcf
            tool_coordinate_frames.append(Frame.from_transformation(t_wcf_tcf))

        return tool_coordinate_frames
