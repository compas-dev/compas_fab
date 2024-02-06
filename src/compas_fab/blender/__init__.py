"""
********************************************************************************
compas_fab.blender
********************************************************************************

.. currentmodule:: compas_fab.blender

Artists
-------

In **COMPAS**, the `artists` are classes that assist with the visualization of
datastructures and models, in a way that maintains the data separated from the
specific CAD interfaces, while providing a way to leverage native performance
of the CAD environment.

"""

try:
    import bpy  # noqa: F401
    import mathutils  # noqa: F401
except ImportError:
    pass
else:
    # TODO: Implement ReachabilityMap support on Blender
    __all__ = []
