from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces.client import forward_docstring
from compas_fab.backends.interfaces.client import PlannerInterface
from compas_fab.backends.pybullet.backend_features.pybullet_add_attached_collision_mesh import PyBulletAddAttachedCollisionMesh
from compas_fab.backends.pybullet.backend_features.pybullet_forward_kinematics import PyBulletForwardKinematics
from compas_fab.backends.pybullet.backend_features.pybullet_inverse_kinematics import PyBulletInverseKinematics
from compas_fab.backends.pybullet.backend_features.pybullet_add_collision_mesh import PyBulletAddCollisionMesh
from compas_fab.backends.pybullet.backend_features.pybullet_append_collision_mesh import PyBulletAppendCollisionMesh
from compas_fab.backends.pybullet.backend_features.pybullet_remove_attached_collision_mesh import PyBulletRemoveAttachedCollisionMesh
from compas_fab.backends.pybullet.backend_features.pybullet_remove_collision_mesh import PyBulletRemoveCollisionMesh


class PyBulletPlanner(PlannerInterface):
    """Implement the planner backend interface for PyBullet."""
    def __init__(self, client):
        super(PyBulletPlanner, self).__init__(client)

    @forward_docstring(PyBulletAddAttachedCollisionMesh)
    def add_attached_collision_mesh(self, *args, **kwargs):
        return PyBulletAddAttachedCollisionMesh(self.client)(*args, **kwargs)

    @forward_docstring(PyBulletAddCollisionMesh)
    def add_collision_mesh(self, *args, **kwargs):
        return PyBulletAddCollisionMesh(self.client)(*args, **kwargs)

    @forward_docstring(PyBulletAppendCollisionMesh)
    def append_collision_mesh(self, *args, **kwargs):
        return PyBulletAppendCollisionMesh(self.client)(*args, **kwargs)

    @forward_docstring(PyBulletRemoveCollisionMesh)
    def remove_collision_mesh(self, *args, **kwargs):
        return PyBulletRemoveCollisionMesh(self.client)(*args, **kwargs)

    @forward_docstring(PyBulletRemoveAttachedCollisionMesh)
    def remove_attached_collision_mesh(self, *args, **kwargs):
        return PyBulletRemoveAttachedCollisionMesh(self.client)(*args, **kwargs)

    @forward_docstring(PyBulletForwardKinematics)
    def forward_kinematics(self, *args, **kwargs):
        return PyBulletForwardKinematics(self.client)(*args, **kwargs)

    @forward_docstring(PyBulletInverseKinematics)
    def inverse_kinematics(self, *args, **kwargs):
        return PyBulletInverseKinematics(self.client)(*args, **kwargs)
