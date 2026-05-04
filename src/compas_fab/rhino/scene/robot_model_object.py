from compas_fab.scene import BaseRobotModelObject

from .rhino_scene_object import RhinoSceneObject


# Overrides any RobotModelObject registered for Rhino by compas_robots.
class RobotModelObject(RhinoSceneObject, BaseRobotModelObject):
    """Scene object for drawing a :class:`~compas_robots.RobotModel` in Rhino.

    Pass-through composition: :class:`BaseRobotModelObject` owns the per-link
    native-geometry cache and the FK delta-transform logic; :class:`RhinoSceneObject`
    supplies the Rhino-doc :meth:`_create_geometry` and :meth:`_transform` hooks.
    """

    pass
