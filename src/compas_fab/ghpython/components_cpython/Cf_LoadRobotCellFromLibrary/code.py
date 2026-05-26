# r: compas_fab>=1.1.0
"""
Load a pre-defined RobotCell (and matching default RobotCellState) from RobotCellLibrary.

COMPAS FAB v1.1.0
"""

import Grasshopper
from compas_ghpython import create_id
from scriptcontext import sticky as st

from compas_fab.robots import RobotCellLibrary


class LoadRobotCellFromLibrary(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, name: str, load_geometry: bool, reload: bool):
        if not name:
            return (None, None)

        name = name.strip().lower()
        load_geometry = True if load_geometry is None else load_geometry

        loader = getattr(RobotCellLibrary, name, None)
        if loader is None:
            available = [
                attr for attr in dir(RobotCellLibrary)
                if not attr.startswith("_") and callable(getattr(RobotCellLibrary, attr))
            ]
            raise ValueError(
                "Unknown RobotCellLibrary entry '{}'. Available: {}".format(name, ", ".join(available))
            )

        key = create_id(ghenv.Component, "robot_cell_{}_{}".format(name, load_geometry))  # noqa: F821
        cached = st.get(key)

        if reload or cached is None:
            robot_cell, robot_cell_state = loader(load_geometry=load_geometry)
            st[key] = (robot_cell, robot_cell_state)
            cached = st[key]

        robot_cell, robot_cell_state = cached
        return (robot_cell, robot_cell_state)
