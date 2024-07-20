from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import compas

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
        print("ClientInterface init")

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

    def from_tcf_to_t0cf(self, tcf_frames, tool_id):
        # type: (List[Frame], str) -> List[Frame]
        """Converts a frame describing the robot's tool coordinate frame (TCF) relative to WCF
        to a frame describing the planner coordinate frame (PCF) (also T0CF), relative to WCF.

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

        tool = self.robot_cell.tool_models[tool_id]
        return tool.from_tcf_to_t0cf(tcf_frames)

    def from_t0cf_to_tcf(self, t0cf_frames, tool_id):
        # type: (List[Frame], str) -> List[Frame]
        """Converts a frame describing the planner coordinate frame (PCF) (also T0CF) relative to WCF
        to a frame describing the robot's tool coordinate frame (TCF) relative to WCF.

        The tool_id must correspond to a valid tool in `planner.robot_cell.tool_models`.

        Parameters
        ----------
        t0cf_frames : list of :class:`~compas.geometry.Frame`
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

        tool = self.robot_cell.tool_models[tool_id]
        return tool.from_t0cf_to_tcf(t0cf_frames)
