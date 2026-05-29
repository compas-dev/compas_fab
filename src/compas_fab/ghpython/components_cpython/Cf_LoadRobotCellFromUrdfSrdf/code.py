# r: compas_fab>=1.1.0
"""
Load a RobotCell from URDF and SRDF files on disk (no ROS required).

Optionally points at a local mesh package folder to load geometry. The
result is sticky-cached keyed on file paths so repeated canvas refreshes
do not re-parse the URDF.

COMPAS FAB v1.1.0
"""

import Grasshopper
from compas_ghpython import create_id
from scriptcontext import sticky as st

from compas_fab.robots import RobotCell


class LoadRobotCellFromUrdfSrdf(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        urdf_filename: str,
        srdf_filename: str,
        local_package_mesh_folder: str,
        reload: bool,
    ):
        if not urdf_filename or not srdf_filename:
            return (None, None)

        key = create_id(  # noqa: F821
            ghenv.Component,  # noqa: F821
            "robot_cell_{}_{}_{}".format(urdf_filename, srdf_filename, local_package_mesh_folder or ""),
        )
        cached = st.get(key)

        if reload or cached is None:
            robot_cell = RobotCell.from_urdf_and_srdf(
                urdf_filename=urdf_filename,
                srdf_filename=srdf_filename,
                local_package_mesh_folder=local_package_mesh_folder or None,
            )
            robot_cell_state = robot_cell.default_cell_state()
            st[key] = (robot_cell, robot_cell_state)
            cached = st[key]

        return cached
