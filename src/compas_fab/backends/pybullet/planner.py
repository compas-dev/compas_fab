from typing import TYPE_CHECKING

from compas_fab.backends.interfaces.planner import PlannerInterface
from compas_fab.backends.pybullet.backend_features import PyBulletCheckCollision
from compas_fab.backends.pybullet.backend_features import PyBulletForwardKinematics
from compas_fab.backends.pybullet.backend_features import PyBulletInverseKinematics
from compas_fab.backends.pybullet.backend_features import PyBulletPlanCartesianMotion
from compas_fab.backends.pybullet.backend_features import PyBulletSetRobotCell
from compas_fab.backends.pybullet.backend_features import PyBulletSetRobotCellState

if TYPE_CHECKING:
    from compas_fab.backends import PyBulletClient

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
        # Initialize all mixins
        super(PyBulletPlanner, self).__init__()

        self._client: PyBulletClient = client
