from compas import IPY

from compas_fab.backends.interfaces.planner import PlannerInterface
from compas_fab.backends.pybullet.backend_features import PyBulletCheckCollision
from compas_fab.backends.pybullet.backend_features import PyBulletForwardKinematics
from compas_fab.backends.pybullet.backend_features import PyBulletInverseKinematics
from compas_fab.backends.pybullet.backend_features import PyBulletPlanCartesianMotion
from compas_fab.backends.pybullet.backend_features import PyBulletSetRobotCell
from compas_fab.backends.pybullet.backend_features import PyBulletSetRobotCellState

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:  # pragma: no cover
        from compas_fab.backends import PyBulletClient  # noqa: F401

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

        self._client = client  # type: PyBulletClient
