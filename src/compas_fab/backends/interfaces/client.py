from compas_robots import RobotModel

from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import RobotSemantics


class ClientInterface:
    """Interface for implementing backend clients.

    Attributes
    ----------
    robot_cell
        The robot cell instance last set on the client.
    robot_cell_state
        The robot cell state instance last set on the client.
    robot_model
        Equivalent to `robot_cell.robot_model`.
    robot_semantics
        Equivalent to `robot_cell.robot_semantics`.
    """

    def __init__(self):
        self._robot_cell = None
        self._robot_cell_state = None

    @property
    def robot_cell(self) -> RobotCell:
        return self._robot_cell

    @property
    def robot_cell_state(self) -> RobotCellState:
        return self._robot_cell_state

    @property
    def robot_model(self) -> RobotModel:
        return self.robot_cell.robot_model

    @property
    def robot_semantics(self) -> RobotSemantics:
        return self.robot_cell.robot_semantics
