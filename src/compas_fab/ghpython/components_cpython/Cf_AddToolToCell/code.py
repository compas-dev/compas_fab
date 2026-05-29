# r: compas_fab>=1.1.0
"""
Add a ToolModel to a RobotCell under a given id.

This is a passthrough builder: the input cell is copied, the tool is added,
and the new cell is returned. The input is not mutated, so wiring multiple
Add* components in series builds up the cell incrementally.

If `tool_id` is empty, the tool's own `.name` is used as id.

COMPAS FAB v1.1.0
"""

from copy import deepcopy

import Grasshopper


class AddToolToCell(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, robot_cell, tool_id: str, tool):
        if robot_cell is None or tool is None:
            return robot_cell

        cell = deepcopy(robot_cell)
        name = tool_id.strip() if tool_id else getattr(tool, "name", None)
        if not name:
            raise ValueError("tool_id is required when the tool has no .name")

        cell.tool_models[name] = tool
        return cell
