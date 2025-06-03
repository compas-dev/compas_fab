# from compas.colors import ColorMap
# from compas.scene import SceneObject
from compas_fab.scene import BaseRobotModelObject

from .gh_scene_object import GHSceneObject


# Overrides the RobotModelObject in compas_robot
class RobotModelObject(GHSceneObject, BaseRobotModelObject):
    """Scene object for drawing a Robot Model in GHPython."""

    pass
