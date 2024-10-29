from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import compas
from compas.plugins import plugin
from compas.scene import register

from compas_fab.robots import RobotCell
from compas_fab.robots import RigidBody
from compas_fab.robots import ReachabilityMap

if compas.RHINO:
    from .reachabilitymapobject import ReachabilityMapObject
    from .rigid_body_object import RigidBodyObject
    from .robot_cell_object import RobotCellObject

    @plugin(category="factories", requires=["Rhino"])
    def register_scene_objects():
        register(RigidBody, RigidBodyObject, context="Grasshopper")
        print("GH RigidBodyObject registered for Grasshopper.")
        register(RobotCell, RobotCellObject, context="Grasshopper")
        print("GH RobotCellObject registered for Grasshopper.")
        register(ReachabilityMap, ReachabilityMapObject, context="Grasshopper")
        print("GH ReachabilityMapObject registered.")


__all__ = [
    "ReachabilityMapObject",
    "RobotCellObject",
    "RigidBodyObject",
]
