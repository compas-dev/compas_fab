from typing import Optional

from compas.colors import Color
from compas.datastructures import Mesh
from compas.geometry import Transformation
from compas_ghpython.drawing import draw_mesh
from compas_ghpython.scene import GHSceneObject as compas_ghpython_GHSceneObject
from compas_rhino.conversions import transformation_to_rhino


class GHSceneObject(compas_ghpython_GHSceneObject):
    """Base class for all GH scene objects."""

    def _transform(self, native_mesh: object, transformation: Transformation) -> object:
        T = transformation_to_rhino(transformation)
        native_mesh.Transform(T)
        return native_mesh

    def _create_geometry(self, geometry: Mesh, name: Optional[str] = None, color: Optional[Color] = None):
        """Create the scene object representing one mesh geometry.

        Parameters
        ----------
        geometry
            Instance of a mesh data structure
        name
            The name of the mesh to draw.
        color
            The color of the object.`

        Returns
        -------
        :rhino:`Rhino.Geometry.Mesh`
        """
        mesh = geometry  # type: Mesh
        color = color.rgba255 if color else None

        vertices, faces = geometry.to_vertices_and_faces(triangulated=False)
        mesh = draw_mesh(vertices, faces, color=color)

        # Try to fix invalid meshes
        if not mesh.IsValid:
            mesh.FillHoles()

        return mesh
