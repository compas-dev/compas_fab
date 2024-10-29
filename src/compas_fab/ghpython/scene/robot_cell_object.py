# from compas.colors import ColorMap
from compas.scene import SceneObject
from compas_fab.scene import BaseRobotCellObject
from .robot_model_object import RobotModelObject
from .rigid_body_object import RigidBodyObject


class RobotCellObject(BaseRobotCellObject):
    """Scene object for drawing a RobotCell in GHPython."""

    def __init__(self, *args, **kwargs):
        super(RobotCellObject, self).__init__(*args, **kwargs)

        # Native Geometry handles
        self.robot_model_scene_object = None  # type: RobotModelObject
        self.robot_model_scene_object = SceneObject(
            item=self.robot_cell.robot_model,
            sceneobject_type=RobotModelObject,
        )

        self.rigid_body_scene_objects = {}  # type: dict[str, RigidBodyObject]
        for id, rigid_body in self.robot_cell.rigid_body_models.items():
            self.rigid_body_scene_objects[id] = SceneObject(
                item=rigid_body,
                sceneobject_type=RigidBodyObject,
            )

        # self.tool_scene_objects = {}  # type: dict[str, BaseToolObject]
        # for id, tool in self.robot_cell.tool_models.items():
        #     self.tool_scene_objects[id] = SceneObject(item=tool)
