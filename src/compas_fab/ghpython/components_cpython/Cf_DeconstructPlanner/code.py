# r: compas_fab>=2.0.1
"""
Deconstruct a planner into the robot cell it currently holds: the
`robot_cell` and its `robot_model`.

These are read from the planner's client (the cell last set on it via a
Load Robot Cell + planner setup). Each output is `None` until a cell has
been loaded. Reads only; never mutates the planner.

The cell *state* is intentionally not exposed: the state held by the client
is whatever was last set internally and is rarely the one you want. Build the
state you need from `Default Cell State` / the `Attach*` components instead.

Works with any planner (MoveIt, PyBullet, Analytical, …).

COMPAS FAB v2.0.1
"""

import Grasshopper
import Rhino
import System


class DeconstructPlanner(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, planner):
        if planner is None:
            return (None, None)

        robot_cell = planner.robot_cell
        robot_model = robot_cell.robot_model if robot_cell is not None else None

        return (robot_cell, robot_model)
