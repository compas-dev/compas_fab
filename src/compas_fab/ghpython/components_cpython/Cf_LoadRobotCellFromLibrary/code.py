# r: compas_fab>=1.1.0
"""
Load a pre-defined RobotCell (and matching default RobotCellState) from RobotCellLibrary.

A dropdown is auto-created on the `name` input listing every available cell.

COMPAS FAB v1.1.0
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import create_id
from compas_ghpython import error
from scriptcontext import sticky as st

from compas_fab.ghpython import ensure_value_list
from compas_fab.robots import RobotCellLibrary

_LIBRARY_NAMES = sorted(
    name
    for name in dir(RobotCellLibrary)
    if not name.startswith("_") and callable(getattr(RobotCellLibrary, name))
)


class LoadRobotCellFromLibrary(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, name: str, load_geometry: bool, reload: bool):
        ensure_value_list(ghenv.Component, "name", _LIBRARY_NAMES, default="ur5")  # noqa: F821

        if not name:
            return (None, None)

        name = name.strip().lower()
        load_geometry = True if load_geometry is None else load_geometry

        loader = getattr(RobotCellLibrary, name, None)
        if loader is None or not callable(loader):
            error(  # noqa: F821
                ghenv.Component,  # noqa: F821
                "Unknown RobotCellLibrary entry '{}'. Available: {}".format(name, ", ".join(_LIBRARY_NAMES)),
            )
            return (None, None)

        key = create_id(ghenv.Component, "robot_cell_{}_{}".format(name, load_geometry))  # noqa: F821
        cached = st.get(key)

        if reload or cached is None:
            try:
                robot_cell, robot_cell_state = loader(load_geometry=load_geometry)
            except Exception as exc:
                # Don't poison the cache; let the user retry by toggling reload or fixing inputs.
                st.pop(key, None)
                error(  # noqa: F821
                    ghenv.Component,  # noqa: F821
                    "Failed to load '{}' (load_geometry={}): {}: {}".format(
                        name, load_geometry, type(exc).__name__, exc
                    ),
                )
                return (None, None)
            st[key] = (robot_cell, robot_cell_state)
            cached = st[key]

        return cached
