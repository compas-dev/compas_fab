# r: compas_fab>=1.1.0
"""
Create a default RobotCellState from a RobotCell.

The default state has the robot at zero configuration with no tools or rigid
bodies attached. Use the cell-state attachment components downstream to wire
in tools and workpieces.

COMPAS FAB v1.1.0
"""

import Grasshopper


class DefaultCellState(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, robot_cell):
        if robot_cell is None:
            return None
        return robot_cell.default_cell_state()
