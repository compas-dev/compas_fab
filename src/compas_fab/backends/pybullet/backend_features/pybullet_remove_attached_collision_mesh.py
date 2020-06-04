from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import pybullet

from compas_fab.backends.backend_feature_interfaces import RemoveAttachedCollisionMesh
from compas_fab.backends.pybullet.utils import LOG


__all__ = [
    'PyBulletRemoveAttachedCollisionMesh',
]


class PyBulletRemoveAttachedCollisionMesh(RemoveAttachedCollisionMesh):
    def __init__(self, client):
        self.client = client

    def remove_attached_collision_mesh(self, id, options={}):
        if id in self.client.attached_collision_objects:
            for constraint_info in self.client.attached_collision_objects[id]:
                pybullet.removeConstraint(constraint_info.constraint_id, physicsClientId=self.client.client_id)
                pybullet.removeBody(constraint_info.body_id, physicsClientId=self.client.client_id)
        else:
            LOG.warning("Attached collision object with name '{}' does not exist in scene.".format(id))
