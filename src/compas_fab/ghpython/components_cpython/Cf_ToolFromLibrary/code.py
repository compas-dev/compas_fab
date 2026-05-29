# r: compas_fab>=1.1.0
"""
Load a pre-defined ToolModel from ToolLibrary.

Supported entries include 'cone', 'printing_tool', 'static_gripper',
'static_gripper_small', and 'kinematic_gripper' (a tool with movable jaws).

COMPAS FAB v1.1.0
"""

import Grasshopper

from compas_fab.robots import ToolLibrary


class ToolFromLibrary(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, name: str, load_geometry: bool):
        if not name:
            return None

        name = name.strip().lower()
        loader = getattr(ToolLibrary, name, None)
        if loader is None:
            available = [attr for attr in dir(ToolLibrary) if not attr.startswith("_") and callable(getattr(ToolLibrary, attr))]
            raise ValueError("Unknown ToolLibrary entry '{}'. Available: {}".format(name, ", ".join(available)))

        load_geometry = True if load_geometry is None else load_geometry
        return loader(load_geometry=load_geometry)
