import compas

from compas_fab.backends.interfaces.client import PlannerInterface

from compas_fab.backends.pybullet.backend_features import *

if not compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from compas_fab.backends import PyBulletClient  # noqa: F401
        from compas_fab.robots import RobotCell
        from compas_fab.robots import RobotCellState

        # Load pybullet for type hinting
        import pybullet

__all__ = [
    "PyBulletPlanner",
]


class PyBulletPlanner(
    PyBulletCheckCollision,
    PyBulletForwardKinematics,
    PyBulletInverseKinematics,
    PyBulletPlanCartesianMotion,
    PyBulletSetRobotCell,
    PyBulletSetRobotCellState,
    PlannerInterface,
):
    """Implement the planner backend interface for PyBullet."""

    def __init__(self, client):
        self._client = client  # type: PyBulletClient

        # Initialize all mixins
        super(PyBulletPlanner, self).__init__()

    # Note that the PyBulletPlanner differs from typical PlannerInterface implementation in that
    # the PyBulletClient keeps the robot cell and robot cell state in memory instead of the planner

    @property
    def robot_cell(self):
        # type: () -> RobotCell

        # The PyBulletClient keeps the robot cell in memory instead of the planner
        return self._client.robot_cell

    @property
    def robot_cell_state(self):
        # type: () -> RobotCellState

        # The PyBulletClient keeps the robot cell state in memory instead of the planner
        return self._client.robot_cell_state
