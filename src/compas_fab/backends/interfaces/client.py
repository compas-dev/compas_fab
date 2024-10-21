from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas import IPY

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from compas_fab.robots import RobotCell  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import RobotSemantics  # noqa: F401
        from compas_robots import RobotModel  # noqa: F401


class ClientInterface(object):
    """Interface for implementing backend clients.

    Attributes
    ----------
    robot : :class:`compas_fab.robots.Robot`, read-only
        The robot instance last associated with the client.
    robot_cell : :class:`compas_fab.robots.RobotCell`, read-only
        The robot cell instance last set on the client.
    robot_cell_state : :class:`compas_fab.robots.RobotCellState`, read-only
        The robot cell state instance last set on the client.
    """

    def __init__(self):
        self._robot_cell = None  # type: RobotCell
        self._robot_cell_state = None  # type: RobotCellState

    @property
    def robot_cell(self):
        # type: () -> RobotCell
        return self._robot_cell

    @property
    def robot_cell_state(self):
        # type: () -> RobotCellState
        return self._robot_cell_state

    @property
    def robot_model(self):
        # type: () -> RobotModel
        return self.robot_cell.robot_model

    @property
    def robot_semantics(self):
        # type: () -> RobotSemantics
        return self.robot_cell.robot_semantics
