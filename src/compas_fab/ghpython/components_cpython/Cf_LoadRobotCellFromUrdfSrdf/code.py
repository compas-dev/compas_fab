# r: compas_fab>=1.1.0
"""
Load a RobotCell from URDF and SRDF files on disk (no ROS required).

Optionally points at a local mesh package folder to load geometry. The
result is sticky-cached keyed on file paths so repeated canvas refreshes
do not re-parse the URDF.

Tools and rigid bodies can be wired directly into `tools` / `rigid_bodies` so the
returned cell already contains them - no separate Add* + set_robot_cell() step is
needed. Each tool/body is registered under its own `.name`.

COMPAS FAB v1.1.0
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import create_id
from scriptcontext import sticky as st

from compas_fab.ghpython import register_models_into_cell
from compas_fab.robots import RobotCell


class LoadRobotCellFromUrdfSrdf(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        urdf_filename: str,
        srdf_filename: str,
        local_package_mesh_folder: str,
        reload: bool,
        tools,
        rigid_bodies,
    ):
        if not urdf_filename or not srdf_filename:
            return (None, None)

        # Cache only the base cell; tools/rigid bodies are merged fresh on every run.
        key = create_id(  # noqa: F821
            ghenv.Component,  # noqa: F821
            "robot_cell_{}_{}_{}".format(urdf_filename, srdf_filename, local_package_mesh_folder or ""),
        )
        base_cell = st.get(key)

        if reload or base_cell is None:
            base_cell = RobotCell.from_urdf_and_srdf(
                urdf_filename=urdf_filename,
                srdf_filename=srdf_filename,
                local_package_mesh_folder=local_package_mesh_folder or None,
            )
            st[key] = base_cell

        robot_cell = register_models_into_cell(ghenv.Component, base_cell, tools, rigid_bodies)  # noqa: F821
        robot_cell_state = robot_cell.default_cell_state()
        return (robot_cell, robot_cell_state)
