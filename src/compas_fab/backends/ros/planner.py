"""
Internal implementation of the planner backend interface for MoveIt!

"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces.client import PlannerInterface

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

__all__ = [
    "MoveItPlanner",
]


class MoveItPlanner(
    MoveItForwardKinematics,
    MoveItInverseKinematics,
    MoveItPlanMotion,
    MoveItPlanCartesianMotion,
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
        self.client = client
