"""Grasshopper (GHPython) scene objects for compas_fab.

Scene objects expose `compas_fab` data structures to Grasshopper while
keeping the data layer separated from CAD-specific code and leveraging
native Rhino/Grasshopper performance.
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
    from .value_list import ensure_value_list

    __all__ = [
        "ReachabilityMapObject",
        "RigidBodyObject",
        "RobotCellObject",
        "RobotModelObject",
        "cache_scene_object",
        "ensure_value_list",
    ]
