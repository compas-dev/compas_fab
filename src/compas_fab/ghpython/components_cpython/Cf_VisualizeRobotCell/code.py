# r: compas_fab>=1.1.0
"""
Draw a RobotCell at the configuration described by a RobotCellState.

This component reuses a cached `RobotCellObject` keyed on the input cell's
identity, so meshes are only built once per canvas session and subsequent
state changes only update transforms (fast).

The robot model, tools, and rigid bodies are returned as separate outputs to
make wiring downstream styling/filtering easy.

COMPAS FAB v1.1.0
"""

import Grasshopper
from compas.scene import SceneObject
from compas_ghpython import create_id
from compas_rhino.conversions import frame_to_rhino_plane
from scriptcontext import sticky as st

from compas_fab.ghpython.scene import RobotCellObject


class VisualizeRobotCell(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        robot_cell,
        cell_state,
        draw_visual: bool,
        draw_collision: bool,
        native_scale: float,
    ):
        if robot_cell is None:
            return (None, None, None, None)

        draw_visual = True if draw_visual is None else draw_visual
        draw_collision = False if draw_collision is None else draw_collision
        native_scale = native_scale if native_scale else 1.0

        key = create_id(  # noqa: F821
            ghenv.Component,  # noqa: F821
            "robot_cell_object_{}_{}_{}_{}".format(id(robot_cell), draw_visual, draw_collision, native_scale),
        )
        scene_object = st.get(key)
        if scene_object is None:
            scene_object = SceneObject(
                item=robot_cell,
                sceneobject_type=RobotCellObject,
                draw_visual=draw_visual,
                draw_collision=draw_collision,
                native_scale=native_scale,
            )
            st[key] = scene_object

        if cell_state is not None:
            scene_object.update(cell_state)

        # Collect outputs from the child scene objects directly so each output
        # can be wired separately without re-running draw().
        robot_meshes = scene_object._robot_model_scene_object.draw()

        tool_meshes = []
        for tool_so in scene_object._tool_scene_objects.values():
            tool_meshes.extend(tool_so.draw())

        rigid_body_meshes = []
        for rb_so in scene_object._rigid_body_scene_objects.values():
            rigid_body_meshes.extend(rb_so.draw())

        base_plane = None
        if cell_state is not None and cell_state.robot_base_frame is not None:
            base_plane = frame_to_rhino_plane(cell_state.robot_base_frame)

        return (robot_meshes, tool_meshes, rigid_body_meshes, base_plane)
