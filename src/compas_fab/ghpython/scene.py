from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import compas
from compas.plugins import plugin
from compas.scene import SceneObject

from compas_fab.robots import ReachabilityMap

if compas.RHINO:
    from .reachabilitymapobject import ReachabilityMapObject

    __all__ = [
        "ReachabilityMapObject",
    ]

    @plugin(category="factories", requires=["Rhino"])
    def register_scene_objects():
        SceneObject.register(ReachabilityMap, ReachabilityMapObject, context="Grasshopper")
