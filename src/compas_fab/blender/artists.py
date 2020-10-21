from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


try:
    import bpy  # noqa: F401
    import mathutils  # noqa: F401
except ImportError:
    pass
else:
    from compas_blender.artists import RobotModelArtist

    __all__ = [
        'RobotModelArtist',
    ]

    # deprecated alias
    RobotArtist = RobotModelArtist
