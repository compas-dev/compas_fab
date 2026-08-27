from compas_fab.backends.interfaces import PlannerInterface

from .backend_features import PyRokiCheckCollision
from .backend_features import PyRokiInverseKinematics
from .backend_features import PyRokiSetRobotCell
from .backend_features import PyRokiSetRobotCellState
from .client import PyRokiClient


class PyRokiPlanner(
    PyRokiSetRobotCell,
    PyRokiSetRobotCellState,
    PyRokiCheckCollision,
    PyRokiInverseKinematics,
    PlannerInterface,
):
    """In-process differentiable kinematics planner backed by PyRoKI."""

    def __init__(self):
        super(PyRokiPlanner, self).__init__()
        self._client = PyRokiClient()
