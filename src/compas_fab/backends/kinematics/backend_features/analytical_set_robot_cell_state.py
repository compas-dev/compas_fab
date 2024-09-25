from compas import IPY

from compas_fab.backends.interfaces import SetRobotCellState

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:

        from compas_fab.robots import RobotCellState  # noqa: F401


class AnalyticalSetRobotCellState(SetRobotCellState):

    def set_robot_cell_state(self, robot_cell_state):
        # type: (RobotCellState) -> None
        """The Analytical Planner does not have collision checking ability, therefore it does not really need
        the robot cell state. This function simply serves as the landing point for other BackendFeature to
        safely call `planner.set_robot_cell_state` without having to check if the planner actually supports it.

        A copy of the RobotCellState object is stored in the client in case it is needed.

        """
        # Update the robot cell in the client
        self.client._robot_cell_state = robot_cell_state.copy()
