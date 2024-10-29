from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


from compas_ghpython.drawing import draw_mesh
from .gh_scene_object import GHSceneObject
from compas_rhino.conversions import transformation_to_rhino


from compas_fab.scene import BaseRigidBodyObject
from compas import IPY

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Optional  # noqa: F401
        from typing import List  # noqa: F401

        from compas.datastructures import Mesh  # noqa: F401
        from compas.geometry import Transformation  # noqa: F401


class RigidBodyObject(GHSceneObject, BaseRigidBodyObject):
    """Scene object for drawing a RigidBody in GHPython."""

    pass
    # def _transform(self, native_mesh, transformation):
    #     # type: (object, Transformation) -> object
    #     T = transformation_to_rhino(transformation)
    #     native_mesh.Transform(T)
    #     return native_mesh

    # def _create_geometry(self, geometry, name=None, color=None):
    #     # type: (Mesh, Optional[str], Optional[List[int]]) -> object
    #     """Create the scene object representing one mesh geometry.

    #     Parameters
    #     ----------
    #     geometry : :class:`~compas.datastructures.Mesh`
    #         Instance of a mesh data structure
    #     name : str, optional
    #         The name of the mesh to draw.
    #     color : :class:`~compas.colors.Color`
    #         The color of the object.`

    #     Returns
    #     -------
    #     :rhino:`Rhino.Geometry.Mesh`
    #     """
    #     mesh = geometry  # type: Mesh
    #     color = color.rgba255 if color else None

    #     vertices, faces = geometry.to_vertices_and_faces(triangulated=False)
    #     mesh = draw_mesh(vertices, faces, color=color)

    #     # Try to fix invalid meshes
    #     if not mesh.IsValid:
    #         mesh.FillHoles()

    #     return mesh
