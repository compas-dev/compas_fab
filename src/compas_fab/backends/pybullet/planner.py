import compas

from compas_fab.backends.interfaces.client import PlannerInterface

from compas_fab.backends.pybullet.backend_features import *

if not compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from compas_fab.backends.interfaces import ClientInterface  # noqa: F401
        from compas_fab.robots import RobotCell  # noqa: F401

        # Load pybullet for type hinting
        import pybullet

__all__ = [
    "PyBulletPlanner",
]


class PyBulletPlanner(
    PyBulletForwardKinematics,
    PyBulletInverseKinematics,
    PyBulletSetRobotCell,
    PyBulletSetRobotCellState,
    PlannerInterface,
):
    """Implement the planner backend interface for PyBullet."""

    def __init__(self, client):
        self._client = client

        # Initialize all mixins
        super(PyBulletPlanner, self).__init__()
