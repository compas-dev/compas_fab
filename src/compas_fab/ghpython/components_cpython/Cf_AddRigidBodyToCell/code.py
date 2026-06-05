# r: compas_fab>=1.1.0
"""
Add a RigidBody to a RobotCell under a given id.

This is a passthrough builder: the input cell is copied, the body is added,
and the new cell is returned. Chain multiple Add* components in series to
build up the cell incrementally.

COMPAS FAB v1.1.0
"""

from copy import deepcopy

import Grasshopper
import Rhino
import System
from compas_ghpython import error


class AddRigidBodyToCell(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, robot_cell, rigid_body_id: str, rigid_body):
        if robot_cell is None or rigid_body is None:
            return robot_cell

        if not rigid_body_id:
            error(ghenv.Component, "rigid_body_id is required")  # noqa: F821
            return robot_cell

        cell = deepcopy(robot_cell)
        cell.rigid_body_models[rigid_body_id.strip()] = rigid_body
        return cell
