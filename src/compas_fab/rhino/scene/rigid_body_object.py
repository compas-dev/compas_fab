from compas_fab.scene import BaseRigidBodyObject

from .rhino_scene_object import RhinoSceneObject


class RigidBodyObject(RhinoSceneObject, BaseRigidBodyObject):
    """Scene object for drawing a :class:`~compas_fab.robots.RigidBody` in Rhino.

    Pass-through composition mirroring :class:`RobotModelObject`: caching and
    delta-transform logic come from :class:`BaseRigidBodyObject`; the Rhino
    bake / transform hooks come from :class:`RhinoSceneObject`.
    """

    pass
