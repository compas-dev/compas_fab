import compas

from compas_fab.backends.interfaces.planner import PlannerInterface

from compas_fab.backends.pybullet.backend_features import *

if not compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from compas_fab.backends import PyBulletClient  # noqa: F401

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
