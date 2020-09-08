from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.vrep.helpers import floats_to_vrep
from compas_fab.backends.interfaces import AddCollisionMesh


__all__ = [
    'VrepAddCollisionMesh',
]


class VrepAddCollisionMesh(AddCollisionMesh):
    """Callable to add a mesh to the 3D scene.
    """
    def __init__(self, client):
        self.client = client

    def add_collision_mesh(self, collision_mesh, options=None):
        """Adds meshes to the 3D scene.

        Args:
            collision_mesh (:obj:`list` of :class:`compas.datastructures.Mesh`): List
                of meshes to add to the current simulation scene.
            options (:obj:`dict`): Unused parameter.

        Returns:
            list: List of object handles (identifiers) assigned to the meshes.

        .. note::
            All meshes are automatically removed from the scene when the simulation ends.
        """
        return self.add_meshes(collision_mesh)

    def add_meshes(self, meshes):
        """Adds meshes to the 3D scene.

        Args:
            meshes (:obj:`list` of :class:`compas.datastructures.Mesh`): List
                of meshes to add to the current simulation scene.

        Returns:
            list: List of object handles (identifiers) assigned to the meshes.

        .. note::
            All meshes are automatically removed from the scene when the simulation ends.
        """
        mesh_handles = []

        for mesh in meshes:
            if not mesh.is_trimesh():
                raise ValueError('The V-REP client only supports tri-meshes')

            vertices, faces = mesh.to_vertices_and_faces()
            vrep_packing = (floats_to_vrep([item for sublist in vertices for item in sublist], self.client.scale) +
                            [item for sublist in faces for item in sublist])
            params = [[len(vertices) * 3, len(faces) * 4], vrep_packing]
            handles = self.client.run_child_script('buildMesh',
                                                   params[0],
                                                   params[1],
                                                   [])[1]
            mesh_handles.extend(handles)
            self.client._added_handles.extend(handles)

        return mesh_handles
