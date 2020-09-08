from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces.client import forward_docstring
from compas_fab.backends.interfaces.client import PlannerInterface
from compas_fab.backends.vrep.backend_features.vrep_add_attached_collision_mesh import VrepAddAttachedCollisionMesh
from compas_fab.backends.vrep.backend_features.vrep_add_collision_mesh import VrepAddCollisionMesh
from compas_fab.backends.vrep.backend_features.vrep_forward_kinematics import VrepForwardKinematics
from compas_fab.backends.vrep.backend_features.vrep_inverse_kinematics import VrepInverseKinematics
from compas_fab.backends.vrep.backend_features.vrep_plan_motion import VrepPlanMotion
from compas_fab.backends.vrep.backend_features.vrep_remove_collision_mesh import VrepRemoveCollisionMesh


class VrepPlanner(PlannerInterface):
    """Implement the planner backend interface for V-REP
    """
    def __init__(self, client):
        super(VrepPlanner, self).__init__(client)

    @forward_docstring(VrepForwardKinematics)
    def forward_kinematics(self, *args, **kwargs):
        return VrepForwardKinematics(self.client)(*args, **kwargs)

    @forward_docstring(VrepInverseKinematics)
    def inverse_kinematics(self, *args, **kwargs):
        return VrepInverseKinematics(self.client)(*args, **kwargs)

    @forward_docstring(VrepPlanMotion)
    def plan_motion(self, *args, **kwargs):
        return VrepPlanMotion(self.client)(*args, **kwargs)

    @forward_docstring(VrepPlanMotion)
    def plan_motion_to_config(self, *args, **kwargs):
        return VrepPlanMotion(self.client).plan_motion_to_config(*args, **kwargs)

    @forward_docstring(VrepAddAttachedCollisionMesh)
    def add_attached_collision_mesh(self, *args, **kwargs):
        return VrepAddAttachedCollisionMesh(self.client)(*args, **kwargs)

    @forward_docstring(VrepAddAttachedCollisionMesh)
    def pick_building_member(self, *args, **kwargs):
        return VrepAddAttachedCollisionMesh(self.client).pick_building_member(*args, **kwargs)

    @forward_docstring(VrepAddCollisionMesh)
    def add_collision_mesh(self, *args, **kwargs):
        return VrepAddCollisionMesh(self.client)(*args, **kwargs)

    @forward_docstring(VrepRemoveCollisionMesh)
    def remove_collision_mesh(self, *args, **kwargs):
        return VrepRemoveCollisionMesh(self.client)(*args, **kwargs)
