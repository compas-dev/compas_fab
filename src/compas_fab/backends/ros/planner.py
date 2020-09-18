"""
Internal implementation of the planner backend interface for MoveIt!
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces.client import forward_docstring
from compas_fab.backends.interfaces.client import PlannerInterface
from compas_fab.backends.ros.backend_features.move_it_add_attached_collision_mesh import MoveItAddAttachedCollisionMesh
from compas_fab.backends.ros.backend_features.move_it_add_collision_mesh import MoveItAddCollisionMesh
from compas_fab.backends.ros.backend_features.move_it_append_collision_mesh import MoveItAppendCollisionMesh
from compas_fab.backends.ros.backend_features.move_it_forward_kinematics import MoveItForwardKinematics
from compas_fab.backends.ros.backend_features.move_it_inverse_kinematics import MoveItInverseKinematics
from compas_fab.backends.ros.backend_features.move_it_plan_cartesian_motion import MoveItPlanCartesianMotion
from compas_fab.backends.ros.backend_features.move_it_plan_motion import MoveItPlanMotion
from compas_fab.backends.ros.backend_features.move_it_planning_scene import MoveItPlanningScene
from compas_fab.backends.ros.backend_features.move_it_remove_attached_collision_mesh import MoveItRemoveAttachedCollisionMesh
from compas_fab.backends.ros.backend_features.move_it_remove_collision_mesh import MoveItRemoveCollisionMesh


class MoveItPlanner(PlannerInterface):
    """Implement the planner backend interface based on MoveIt!
    """

    def __init__(self, client):
        super(MoveItPlanner, self).__init__(client)

    @forward_docstring(MoveItForwardKinematics)
    def forward_kinematics(self, *args, **kwargs):
        return MoveItForwardKinematics(self.client)(*args, **kwargs)

    @forward_docstring(MoveItInverseKinematics)
    def inverse_kinematics(self, *args, **kwargs):
        return MoveItInverseKinematics(self.client)(*args, **kwargs)

    @forward_docstring(MoveItPlanCartesianMotion)
    def plan_cartesian_motion(self, *args, **kwargs):
        return MoveItPlanCartesianMotion(self.client)(*args, **kwargs)

    @forward_docstring(MoveItPlanMotion)
    def plan_motion(self, *args, **kwargs):
        return MoveItPlanMotion(self.client)(*args, **kwargs)

    @forward_docstring(MoveItPlanningScene)
    def get_planning_scene(self, *args, **kwargs):
        return MoveItPlanningScene(self.client)(*args, **kwargs)

    @forward_docstring(MoveItAddCollisionMesh)
    def add_collision_mesh(self, *args, **kwargs):
        return MoveItAddCollisionMesh(self.client)(*args, **kwargs)

    @forward_docstring(MoveItRemoveCollisionMesh)
    def remove_collision_mesh(self, *args, **kwargs):
        return MoveItRemoveCollisionMesh(self.client)(*args, **kwargs)

    @forward_docstring(MoveItAppendCollisionMesh)
    def append_collision_mesh(self, *args, **kwargs):
        return MoveItAppendCollisionMesh(self.client)(*args, **kwargs)

    @forward_docstring(MoveItAddAttachedCollisionMesh)
    def add_attached_collision_mesh(self, *args, **kwargs):
        return MoveItAddAttachedCollisionMesh(self.client)(*args, **kwargs)

    @forward_docstring(MoveItRemoveAttachedCollisionMesh)
    def remove_attached_collision_mesh(self, *args, **kwargs):
        return MoveItRemoveAttachedCollisionMesh(self.client)(*args, **kwargs)
