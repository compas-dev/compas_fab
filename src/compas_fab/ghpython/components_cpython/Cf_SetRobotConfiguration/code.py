# r: compas_fab>=2.0.0
"""
Return a new RobotCellState with `robot_configuration` overridden.

This is the common bridge between planning outputs (which produce a
Configuration) and visualization / further planning (which consume a
RobotCellState). The input state is not mutated.

COMPAS FAB v2.0.0
"""

import Grasshopper
import Rhino
import System


class SetRobotConfiguration(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, cell_state, configuration):
        if cell_state is None:
            return None
        if configuration is None:
            return cell_state

        new_state = cell_state.copy()
        new_state.robot_configuration = configuration
        return new_state
