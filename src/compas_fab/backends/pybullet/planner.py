from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces.client import PlannerInterface

from compas_fab.backends.pybullet.backend_features.pybullet_add_attached_collision_mesh import (
    PyBulletAddAttachedCollisionMesh,
)
from compas_fab.backends.pybullet.backend_features.pybullet_add_collision_mesh import PyBulletAddCollisionMesh
from compas_fab.backends.pybullet.backend_features.pybullet_append_collision_mesh import PyBulletAppendCollisionMesh
from compas_fab.backends.pybullet.backend_features.pybullet_forward_kinematics import PyBulletForwardKinematics
from compas_fab.backends.pybullet.backend_features.pybullet_inverse_kinematics import PyBulletInverseKinematics
from compas_fab.backends.pybullet.backend_features.pybullet_remove_attached_collision_mesh import (
    PyBulletRemoveAttachedCollisionMesh,
)
from compas_fab.backends.pybullet.backend_features.pybullet_remove_collision_mesh import PyBulletRemoveCollisionMesh

__all__ = [
    "PyBulletPlanner",
]


class PyBulletPlanner(
    PyBulletAddAttachedCollisionMesh,
    PyBulletAddCollisionMesh,
    PyBulletAppendCollisionMesh,
    PyBulletRemoveCollisionMesh,
    PyBulletRemoveAttachedCollisionMesh,
    PyBulletForwardKinematics,
    PyBulletInverseKinematics,
    PlannerInterface,
):
    """Implement the planner backend interface for PyBullet."""

    def __init__(self, client):
        self._client = client

        # Initialize all mixins
        super(PyBulletPlanner, self).__init__()
