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
from compas_fab.backends.ros.backend_features.move_it_add_attached_collision_mesh import MoveItAddAttachedCollisionMesh
from compas_fab.backends.ros.backend_features.move_it_add_collision_mesh import MoveItAddCollisionMesh
from compas_fab.backends.ros.backend_features.move_it_append_collision_mesh import MoveItAppendCollisionMesh
from compas_fab.backends.ros.backend_features.move_it_forward_kinematics import MoveItForwardKinematics
from compas_fab.backends.ros.backend_features.move_it_inverse_kinematics import MoveItInverseKinematics
from compas_fab.backends.ros.backend_features.move_it_plan_cartesian_motion import MoveItPlanCartesianMotion
from compas_fab.backends.ros.backend_features.move_it_plan_motion import MoveItPlanMotion
from compas_fab.backends.ros.backend_features.move_it_planning_scene import MoveItPlanningScene
from compas_fab.backends.ros.backend_features.move_it_remove_attached_collision_mesh import (
    MoveItRemoveAttachedCollisionMesh,
)
from compas_fab.backends.ros.backend_features.move_it_remove_collision_mesh import MoveItRemoveCollisionMesh
from compas_fab.backends.ros.messages import PlanningSceneWorld
from compas_fab.backends.ros.messages import PlanningScene
from compas_fab.backends.ros.messages import RobotState


from compas import IPY

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Optional  # noqa: F401
        from typing import Dict

__all__ = [
    "MoveItPlanner",
]


class MoveItPlanner(
    MoveItForwardKinematics,
    MoveItInverseKinematics,
    MoveItPlanMotion,
    MoveItPlanCartesianMotion,
    MoveItSetRobotCell,
    MoveItSetRobotCellState,
    MoveItPlanningScene,
    MoveItResetPlanningScene,
    MoveItAddCollisionMesh,
    MoveItRemoveCollisionMesh,
    MoveItAppendCollisionMesh,
    MoveItAddAttachedCollisionMesh,
    MoveItRemoveAttachedCollisionMesh,
    PlannerInterface,
):
    """Implement the planner backend interface based on MoveIt!"""

    def __init__(self, client):
        self._client = client

        # Initialize all mixins
        super(MoveItPlanner, self).__init__()

        self._last_planning_scene_world = None  # type: Optional[PlanningSceneWorld]
        self._last_robot_state = None  # type: Optional[RobotState]

        self._current_rigid_body_hashes = {}  # type: Dict[str, bytes]

    @property
    def last_planning_scene_world(self):
        # type: () -> PlanningSceneWorld
        if self._last_planning_scene_world is None:
            planning_scene = self.get_planning_scene()  # type: PlanningScene
            self._last_planning_scene_world = planning_scene.world
            self._last_robot_state = planning_scene.robot_state
        return self._last_planning_scene_world

    @property
    def last_robot_state(self):
        # type: () -> RobotState
        if self._last_robot_state is None:
            planning_scene = self.get_planning_scene()  # type: PlanningScene
            self._last_planning_scene_world = planning_scene.world
            self._last_robot_state = planning_scene.robot_state
        return self._last_robot_state
