"""Rhino-context scene-object plugin for compas_fab.

Mirrors the GHPython plugin in :mod:`compas_fab.ghpython.scene` so a Rhino
host can call ``Scene().add(robot_cell)`` and get a :class:`RobotCellObject`
that bakes geometry once and updates it in place across state changes.

Adding a new SceneObject for an existing item type (e.g. ``RobotCell``)
under a *different* context name does not conflict with any other plugin;
the COMPAS plugin system keys on ``(item_type, context)``.
"""

import compas
from compas.plugins import plugin
from compas.scene import register

from compas_fab.robots import ReachabilityMap
from compas_fab.robots import RigidBody
from compas_fab.robots import RobotCell

# ReachabilityMapObject was previously the sole Rhino scene object exposed
# at ``compas_fab.rhino.scene.ReachabilityMapObject``; keep that import path
# stable by re-exporting it from the package __init__.
from ..reachabilitymapobject import ReachabilityMapObject

if compas.RHINO:
    from .rigid_body_object import RigidBodyObject
    from .robot_cell_object import RobotCellObject
    from .robot_model_object import RobotModelObject

    @plugin(category="factories", requires=["Rhino"])
    def register_scene_objects():
        # NOTE: must use the `register` *function* imported from
        # `compas.scene` (== `compas.scene.context.register`).  There is NO
        # `SceneObject.register` classmethod; calling it raises
        # AttributeError, which the plugin manager swallows silently and the
        # registration never lands -> `Scene().add(robot_cell)` then fails
        # with `SceneObjectNotRegisteredError`.
        register(ReachabilityMap, ReachabilityMapObject, context="Rhino")
        register(RigidBody, RigidBodyObject, context="Rhino")
        register(RobotCell, RobotCellObject, context="Rhino")


__all__ = [
    "ReachabilityMapObject",
    "RigidBodyObject",
    "RobotCellObject",
    "RobotModelObject",
]
