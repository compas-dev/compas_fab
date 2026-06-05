# r: compas_fab>=1.1.0
"""
Add a ToolModel to a RobotCell and attach it to a planning group in one step.

Shortcut for the common case of `AddToolToCell` + `DefaultCellState` +
`AttachToolToRobot` wired in series. For workflows that add many tools and
attach a subset, keep using the three-component chain instead.

The tool's id in `cell.tool_models` defaults to `tool.name`; override via
`tool_id`. The planning group defaults to the cell's main group. The
`touch_links` list defaults to the group's end-effector link (plus its
parent if the EE link itself has no geometry, e.g. UR `tool0`).

A remark announces the link the tool ends up attached to so the chosen
end-effector frame is never opaque. Use `attachment_plane` to rotate or
offset the tool's `tool0` frame relative to that link (this is how you
correct for any per-robot EE-axis convention quirks).

COMPAS FAB v1.1.0
"""

from copy import deepcopy

import Grasshopper
import Rhino
import System
from compas.geometry import Frame
from compas_ghpython import error
from compas_ghpython import remark
from compas_ghpython import warning
from compas_rhino.conversions import plane_to_compas_frame


class AddAndAttachTool(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        robot_cell,
        tool,
        tool_id: str,
        group: str,
        attachment_plane,
        touch_links: System.Collections.Generic.List[str],
    ):
        if robot_cell is None or tool is None:
            return (robot_cell, None)

        name = tool_id.strip() if tool_id else getattr(tool, "name", None)
        if not name:
            error(ghenv.Component, "tool_id is required when the tool has no .name")  # noqa: F821
            return (robot_cell, None)

        group = group.strip() if group else robot_cell.main_group_name

        attachment_frame = None
        if attachment_plane is not None:
            attachment_frame = attachment_plane if isinstance(attachment_plane, Frame) else plane_to_compas_frame(attachment_plane)

        touch_links_list = list(touch_links) if touch_links else None
        if touch_links_list is None:
            touch_links_list = robot_cell.default_touch_links(group)
            warning(ghenv.Component, "touch_links unwired; defaulted to {} (links the tool inevitably overlaps). Wire your own list to override.".format(touch_links_list))  # noqa: F821

        ee_link = robot_cell.get_end_effector_link_name(group)
        remark(  # noqa: F821
            ghenv.Component,  # noqa: F821
            "attaching '{}' at link '{}' (group '{}' end-effector). The tool's `tool0` frame is at this link; "
            "use `attachment_plane` to rotate/offset the tool relative to it.".format(name, ee_link, group),
        )

        new_cell = deepcopy(robot_cell)
        new_cell.tool_models[name] = tool

        new_state = new_cell.default_cell_state()
        new_state.set_tool_attached_to_group(
            tool_id=name,
            group=group,
            attachment_frame=attachment_frame,
            touch_links=touch_links_list,
        )
        return (new_cell, new_state)
