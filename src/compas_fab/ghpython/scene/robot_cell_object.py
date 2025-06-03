from compas.scene import SceneObject

from compas_fab.scene import BaseRobotCellObject
from .rigid_body_object import RigidBodyObject
from .robot_model_object import RobotModelObject


class RobotCellObject(BaseRobotCellObject):
    """Scene object for drawing a RobotCell in GHPython."""

    def _initial_draw(self) -> None:

        kwargs = {
            "draw_visual": self._draw_visual,
            "draw_collision": self._draw_collision,
            "native_scale": self._native_scale,
        }

        # RobotModel drawn using RobotModelObject from compas_fab.ghpython not compas_robots
        self._robot_model_scene_object : RobotModelObject = None
        self._robot_model_scene_object = SceneObject(
            item=self.robot_cell.robot_model,
            sceneobject_type=RobotModelObject,
            **kwargs,
        )

        # RigidBodies
        self._rigid_body_scene_objects : dict[str, RigidBodyObject] = {}
        for id, rigid_body in self.robot_cell.rigid_body_models.items():
            self._rigid_body_scene_objects[id] = SceneObject(
                item=rigid_body,
                sceneobject_type=RigidBodyObject,
                **kwargs,
            )

        # ToolModels
        # NOTE: Tools are also drawn using a RobotModelObject
        self._tool_scene_objects : dict[str, RobotModelObject] = {}
        for id, tool in self.robot_cell.tool_models.items():
            self._tool_scene_objects[id] = SceneObject(
                item=tool,
                sceneobject_type=RobotModelObject,
                **kwargs,
            )
