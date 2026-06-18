# r: compas_fab>=2.0.0
"""
Load a pre-defined ToolModel from ToolLibrary.

Supported entries include 'cone', 'printing_tool', 'static_gripper',
'static_gripper_small', and 'kinematic_gripper' (a tool with movable jaws).

Also returns the tool's visual meshes as Rhino geometry for preview, drawn
in the tool's own base frame (not yet attached to any robot).

COMPAS FAB v2.0.0
"""

import Grasshopper
import Rhino
import System
from compas.scene import SceneObject
from compas_ghpython import error

from compas_fab.ghpython.scene import RobotModelObject
from compas_fab.robots import ToolLibrary


class ToolFromLibrary(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, name: str, load_geometry: bool):
        if not name:
            return (None, None)

        name = name.strip().lower()
        loader = getattr(ToolLibrary, name, None)
        if loader is None:
            available = [attr for attr in dir(ToolLibrary) if not attr.startswith("_") and callable(getattr(ToolLibrary, attr))]
            error(ghenv.Component, "Unknown ToolLibrary entry '{}'. Available: {}".format(name, ", ".join(available)))  # noqa: F821
            return (None, None)

        load_geometry = True if load_geometry is None else load_geometry
        tool = loader(load_geometry=load_geometry)

        visual_meshes = []
        if load_geometry:
            scene_object = SceneObject(
                item=tool,
                sceneobject_type=RobotModelObject,
                draw_visual=True,
                draw_collision=False,
            )
            visual_meshes = scene_object.draw()

        return (tool, visual_meshes)
