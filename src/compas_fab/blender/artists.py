from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_blender import draw_mesh

from compas_fab.artists import BaseRobotArtist

try:
    import bpy
    import mathutils
except ImportError:
    pass

__all__ = [
    'RobotArtist',
]


class RobotArtist(BaseRobotArtist):
    """Visualizer for robots inside a Blender environment."""

    def __init__(self, robot, layer=None):
        self.layer = layer
        super(RobotArtist, self).__init__(robot)

    def transform(self, native_mesh, transformation):
        native_mesh.matrix_world @= mathutils.Matrix(transformation.matrix)

    def draw_geometry(self, geometry, name=None, color=None):
        # Imported colors take priority over a the parameter color
        if 'mesh_color.diffuse' in geometry.attributes:
            color = geometry.attributes['mesh_color.diffuse']

        # If we have a color, we'll discard alpha because draw_mesh is hard coded for a=1
        if color:
            r, g, b, _a = color
            color = [r, g, b]
        else:
            color = [1., 1., 1.]
        print('color', color)

        if self.layer:
            collection = bpy.data.collections.new(self.layer)
            bpy.context.scene.collection.children.link(collection)

        v, f = geometry.to_vertices_and_faces()
        return draw_mesh(vertices=v, faces=f, name=name, color=color, centroid=False, layer=self.layer)

    def redraw(self, timeout=None):
        bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
