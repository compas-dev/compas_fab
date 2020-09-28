from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


try:
    import bpy
    import mathutils
except ImportError:
    pass
else:
    from compas_blender import draw_mesh
    from compas_blender.artists import RobotModelArtist

__all__ = [
    'RobotModelArtist',
]

# deprecated alias
RobotArtist = RobotModelArtist
