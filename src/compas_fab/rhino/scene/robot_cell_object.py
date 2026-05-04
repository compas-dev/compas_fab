"""Composite scene object for drawing a :class:`~compas_fab.robots.RobotCell` in Rhino.

Mirrors :class:`compas_fab.ghpython.scene.RobotCellObject` but builds child
scene objects against the Rhino-context :class:`RobotModelObject` /
:class:`RigidBodyObject` so the cell's geometry can be baked once into the
active document and then updated in place via delta transforms.

The constructor accepts a ``draw_rigid_bodies`` flag (default ``True``) so
callers that only want robot + tools (e.g. the IK keyframe preview, where
bars/joints already exist as separately-managed Rhino geometry) can skip
the rigid-body children without subclassing.
"""

from typing import Optional

from compas.scene import SceneObject

from compas_fab.scene import BaseRobotCellObject

from .rigid_body_object import RigidBodyObject
from .robot_model_object import RobotModelObject


class RobotCellObject(BaseRobotCellObject):
    """Scene object for drawing a :class:`~compas_fab.robots.RobotCell` in Rhino."""

    def __init__(
        self,
        draw_visual: bool = True,
        draw_collision: bool = False,
        native_scale: float = 1.0,
        draw_rigid_bodies: bool = True,
        layer: Optional[str] = None,
        *args,
        **kwargs,
    ):
        super(RobotCellObject, self).__init__(
            draw_visual=draw_visual,
            draw_collision=draw_collision,
            native_scale=native_scale,
            *args,
            **kwargs,
        )
        # Stored so child scene objects created in `_initial_draw` inherit it.
        self._draw_rigid_bodies = draw_rigid_bodies
        self._layer = layer

    def _initial_draw(self) -> None:
        kwargs = {
            "draw_visual": self._draw_visual,
            "draw_collision": self._draw_collision,
            "native_scale": self._native_scale,
        }
        if self._layer is not None:
            kwargs["layer"] = self._layer

        # RobotModel — drawn via our Rhino RobotModelObject (not the one
        # compas_robots may register), so all per-link geometry goes through
        # the cached + delta-transform path.
        self._robot_model_scene_object = SceneObject(
            item=self.robot_cell.robot_model,
            sceneobject_type=RobotModelObject,
            **kwargs,
        )

        # RigidBodies — optional; skipped when ``draw_rigid_bodies=False``.
        self._rigid_body_scene_objects: dict = {}
        if self._draw_rigid_bodies:
            for id, rigid_body in self.robot_cell.rigid_body_models.items():
                self._rigid_body_scene_objects[id] = SceneObject(
                    item=rigid_body,
                    sceneobject_type=RigidBodyObject,
                    **kwargs,
                )

        # ToolModels — drawn via RobotModelObject (a ToolModel is a RobotModel).
        self._tool_scene_objects: dict = {}
        for id, tool in self.robot_cell.tool_models.items():
            self._tool_scene_objects[id] = SceneObject(
                item=tool,
                sceneobject_type=RobotModelObject,
                **kwargs,
            )
