from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import time

import compas
from compas_blender import xdraw_mesh

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

    def __init__(self, robot):
        super(RobotArtist, self).__init__(robot)

    def transform(self, native_mesh, transformation):
        native_mesh.matrix_world *= mathutils.Matrix(transformation.matrix)

    def draw_mesh(self, compas_mesh):
        v, f = compas_mesh.to_vertices_and_faces()
        return xdraw_mesh(v, f)
