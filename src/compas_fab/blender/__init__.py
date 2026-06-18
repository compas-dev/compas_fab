"""Blender bindings for compas_fab. Only imports when run inside Blender."""

try:
    import bpy  # noqa: F401
    import mathutils  # noqa: F401
except ImportError:
    pass
else:
    # TODO: Implement ReachabilityMap support on Blender
    __all__ = []
