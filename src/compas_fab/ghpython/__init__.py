"""
********************************************************************************
compas_fab.ghpython
********************************************************************************

.. currentmodule:: compas_fab.ghpython

Scene objects
-------------

In **COMPAS**, the `scene objects` are classes that assist with the visualization
of datastructures and models, in a way that maintains the data separated from the
specific CAD interfaces, while providing a way to leverage native performance
of the CAD environment.

.. autosummary::
    :toctree: generated/
    :nosignatures:

    ReachabilityMapObject
    RigidBodyObject
    RobotCellObject
    RobotModelObject


"""

import compas

if compas.RHINO:
    from .scene import (
        ReachabilityMapObject,
        RigidBodyObject,
        RobotCellObject,
        RobotModelObject,
    )
    from .sticky_cache import cache_scene_object

    __all__ = [
        "ReachabilityMapObject",
        "RigidBodyObject",
        "RobotCellObject",
        "RobotModelObject",
        "cache_scene_object",
    ]
