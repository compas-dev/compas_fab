from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas import IPY

from compas_fab.backends.exceptions import BackendFeatureNotSupportedError


if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:  # pragma: no cover
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
        self._client = client
        super(PlannerInterface, self).__init__()

    @property
    def client(self):
        # type: () -> ClientInterface
        return self._client

    # ===========================================================================
    # Below is a list of methods offered by the mixin classes of PlannerInterface
    # The actual implementation of these methods can be found in the backend
    # specific planner classes. e.g.:
    # - src/compas_fab/backends/ros/backend_features/move_it_set_robot_cell.py
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

    # TODO: Improve the docstring for the methods below

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

    def iter_inverse_kinematics(self, *args, **kwargs):
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

    # ==========================================================================
    # Sanity Check Functions
    # ==========================================================================

    # Nothing here yet.
