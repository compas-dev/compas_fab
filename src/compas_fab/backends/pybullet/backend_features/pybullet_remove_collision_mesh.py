from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import pybullet

from compas_fab.backends.backend_feature_interfaces import RemoveCollisionMesh
from compas_fab.backends.pybullet.utils import LOG


__all__ = [
    'PyBulletRemoveCollisionMesh',
]


class PyBulletRemoveCollisionMesh(RemoveCollisionMesh):
    def __init__(self, client):
        self.client = client

    def remove_collision_mesh(self, id, options={}):
        if id in self.client.collision_objects:
            for body_id in self.client.collision_objects[id]:
                pybullet.removeBody(body_id)
        else:
            LOG.warning("Collision object with name '{}' does not exist in scene.".format(id))
