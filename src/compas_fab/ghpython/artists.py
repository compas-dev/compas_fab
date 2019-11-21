from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.utilities import rgb_to_rgb

from compas_ghpython import mesh_draw
from compas_ghpython.geometry import xform_from_transformation

from compas_fab.artists import BaseRobotArtist

__all__ = [
    'RobotArtist',
]


class RobotArtist(BaseRobotArtist):
    """Visualizer for robots inside a Grasshopper environment."""

    def __init__(self, robot):
        super(RobotArtist, self).__init__(robot)

    def transform(self, native_mesh, transformation):
        T = xform_from_transformation(transformation)
        native_mesh.Transform(T)

    def draw_geometry(self, geometry, name=None, color=None):
        if color:
            color = rgb_to_rgb(color[0], color[1], color[2])

        mesh = mesh_draw(geometry, color=color)

        # Try to fix invalid meshes
        if not mesh.IsValid:
            mesh.FillHoles()

        return mesh
