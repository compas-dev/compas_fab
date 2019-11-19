from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_blender import draw_mesh

from compas_fab.artists import BaseRobotArtist

try:
    import mathutils
except ImportError:
    pass

__all__ = [
    'RobotArtist',
]


class RobotArtist(BaseRobotArtist):
    """Visualizer for robots inside a Blender environment."""

    def __init__(self, robot, layer=None):
        super(RobotArtist, self).__init__(robot)
        self.layer = layer

    def transform(self, native_mesh, transformation):
        native_mesh.matrix_world *= mathutils.Matrix(transformation.matrix)

    def draw_geometry(self, geometry, name=None, color=None):
        v, f = geometry.to_vertices_and_faces()
        return draw_mesh(v, f, name=name, color=color, layer=self.layer)
