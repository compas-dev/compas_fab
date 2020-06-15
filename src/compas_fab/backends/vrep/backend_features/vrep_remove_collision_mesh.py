from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.vrep.helpers import DEFAULT_OP_MODE
from compas_fab.backends.vrep.remote_api import vrep
from compas_fab.backends.interfaces import RemoveCollisionMesh


__all__ = [
    'VrepRemoveCollisionMesh',
]


class VrepRemoveCollisionMesh(RemoveCollisionMesh):
    """Callable to remove collision meshes from the 3D scene.
    """
    def __init__(self, client):
        self.client = client

    def remove_collision_mesh(self, id, options=None):
        """Removes objects from the 3D scene.

        Args:
            id (:obj:`list` of :obj:`int`) : Object handles to remove.

        .. note::
            Please note there's no need to clean up objects manually after the simulation
            has completed, as those will be reset automatically anyway. This method is
            only useful if you need to remove objects *during* a simulation.
        """
        self.remove_objects(id)

    def remove_meshes(self, mesh_handles):
        """Removes meshes from the 3D scene.

        This is functionally identical to ``remove_objects``, but it's here for
        symmetry reasons.

        Args:
            mesh_handles (:obj:`list` of :obj:`int`): Object handles to remove.
        """
        self.remove_objects(mesh_handles)

    def remove_objects(self, object_handles):
        """Removes objects from the 3D scene.

        Args:
            object_handles (:obj:`list` of :obj:`int`): Object handles to remove.

        .. note::
            Please note there's no need to clean up objects manually after the simulation
            has completed, as those will be reset automatically anyway. This method is
            only useful if you need to remove objects *during* a simulation.
        """
        for handle in object_handles:
            vrep.simxRemoveObject(self.client.client_id, handle, DEFAULT_OP_MODE)

        self.client._added_handles = filter(lambda x: x not in object_handles, self.client._added_handles)
