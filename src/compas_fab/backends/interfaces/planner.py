from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas import IPY

from compas_fab.backends.exceptions import BackendFeatureNotSupportedError
from compas_fab.backends.exceptions import TargetModeMismatchError
from compas_fab.robots import TargetMode

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import List  # noqa: F401

        from compas_fab.backends.interfaces import ClientInterface  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401


class PlannerInterface(object):
    """Interface for implementing backend planners.

    Planner is connected to a ClientInterface when initiated, and can access
    the client's robot instance.
    It is responsible for all planning services for the robot.

    Parameters
    ----------
    client : :class:`compas_fab.backends.interfaces.ClientInterface`, optional
        The client instance associated with the planner.
        Typically this is set by the backend client during initialization
        using `planner = PlannerInterface(client)`.
        Consult the chosen backend client for how to set this attribute.

    Attributes
    ----------
    client : :class:`compas_fab.backends.interfaces.ClientInterface`, read-only
        The client instance associated with the planner.
        It cannot be changed after initialization.
        The client also keeps the `.robot_cell` and `.robot_cell_state` in memory.

    """

    def __init__(self, client=None):
        if client:
            self._client = client
        super(PlannerInterface, self).__init__()

    @property
    def client(self):
        # type: () -> ClientInterface
        return self._client

    # ===========================================================================
    # Below is a list of methods offered by the mixin classes of PlannerInterface
    # ===========================================================================

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
    # Planning Services
    # ==========================================================================

    def check_collisions(self, *args, **kwargs):
        """Default method for planner.

        Raises
        ------
        BackendFeatureNotSupportedError
            Planner does not have this feature.
        """
        raise BackendFeatureNotSupportedError("Assigned planner does not have this feature.")

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
    # Sanity Check Functions
    # ==========================================================================

    def ensure_robot_cell_state_supports_target_mode(self, robot_cell_state, target_mode, group):
        # type: (RobotCellState, TargetMode | None, str) -> None
        """Check if the `target_mode` agrees with the attachment state as specified in the `robot_cell_state`.

        If the `target_mode` is None,
        such as the cases for ConfigurationTarget and ConstraintSetTarget,
        this function will not perform any checks.

        An `robot_cell_state` input is used instead of the `self.client.robot_cell_state` attribute,
        to allow the check to be performed before calling `set_robot_cell_state()`.

        If target mode is `TargetMode.TOOL`, the specified planning group must have a tool attached.
        If target mode is `TargetMode.WORKPIECE`, the specified planning group must have one and only one workpiece attached.

        Parameters
        ----------
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`
            The robot cell state to check.
        target_mode : :class:`compas_fab.robots.TargetMode` or None
            The target or waypoints to check.
        group : str
            The planning group to check. Must be specified.

        Raises
        ------
        TargetModeMismatchError
            If the target mode is `TOOL` and no tool is attached to the robot in the specified group.
            If the target mode is `WORKPIECE` and no (or more than one) workpiece is attached to the specified group.
        """

        if target_mode is None:
            return

        assert group is not None, "Planning group must be specified."

        # Checks for Tool Mode
        tool_id = robot_cell_state.get_attached_tool_id(group)
        if target_mode == TargetMode.TOOL:

            if tool_id is None:
                raise TargetModeMismatchError(
                    "Target mode is 'TOOL', but no tool is attached to the robot in group '{}'.".format(group)
                )

        # Checks for Workpiece Mode
        workpiece_ids = robot_cell_state.get_attached_workpiece_ids(group)
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
