# r: compas_fab>=2.0.0
"""
Deconstruct a RobotCell into its parts: the robot model, the registered
rigid bodies and tools, and a matching default RobotCellState.

`rigid_bodies` and `tools` are the registered models (the values of
`cell.rigid_body_models` / `cell.tool_models`), in no particular order.
`default_cell_state` is the same state `Load Robot Cell` returns - every
tool/body present but nothing attached - handy as a starting point to wire
into the Attach* components.

This is the inverse of registering models via the `Load Robot Cell`
components; it reads the cell and never mutates it.

COMPAS FAB v2.0.0
"""

import Grasshopper
import Rhino
import System


class DeconstructRobotCell(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, robot_cell):
        if robot_cell is None:
            return (None, [], [], None)

        robot_model = robot_cell.robot_model
        rigid_bodies = list(robot_cell.rigid_body_models.values())
        tools = list(robot_cell.tool_models.values())
        default_cell_state = robot_cell.default_cell_state()

        return (robot_model, rigid_bodies, tools, default_cell_state)
