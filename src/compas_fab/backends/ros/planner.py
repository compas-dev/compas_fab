"""
Internal implementation of the planner backend interface for MoveIt!

"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces.planner import PlannerInterface

from compas_fab.backends.ros.backend_features import MoveItSetRobotCell
from compas_fab.backends.ros.backend_features import MoveItSetRobotCellState
from compas_fab.backends.ros.backend_features import MoveItResetPlanningScene
from compas_fab.backends.ros.backend_features.move_it_forward_kinematics import MoveItForwardKinematics
from compas_fab.backends.ros.backend_features.move_it_inverse_kinematics import MoveItInverseKinematics
from compas_fab.backends.ros.backend_features.move_it_plan_cartesian_motion import MoveItPlanCartesianMotion
from compas_fab.backends.ros.backend_features.move_it_plan_motion import MoveItPlanMotion
from compas_fab.backends.ros.backend_features.move_it_planning_scene import MoveItPlanningScene


from compas import IPY

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:  # pragma: no cover
        from typing import Dict  # noqa: F401

__all__ = [
    "MoveItPlanner",
]


class MoveItPlanner(
    MoveItPlanningScene,
    MoveItResetPlanningScene,
    MoveItSetRobotCell,
    MoveItSetRobotCellState,
    MoveItForwardKinematics,
    MoveItInverseKinematics,
    MoveItPlanMotion,
    MoveItPlanCartesianMotion,
    PlannerInterface,
):
    """Implement the planner backend interface based on MoveIt!"""

    def __init__(self, client):

        # Initialize all mixins
        super(MoveItPlanner, self).__init__()

        self._client = client
        self._current_rigid_body_hashes = {}  # type: Dict[str, bytes]
        self._current_tool_hashes = {}  # type: Dict[str, bytes]

        # Reset the planning scene in the backend to clear all objects left by previous runs
        self.reset_planning_scene()
