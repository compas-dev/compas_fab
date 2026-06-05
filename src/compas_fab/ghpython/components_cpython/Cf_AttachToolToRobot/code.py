# r: compas_fab>=1.1.0
"""
Attach a tool to a robot planning group inside a RobotCellState.

Passthrough builder: the input state is copied, the attachment is set, and
the new state is returned.

When `robot_cell` is wired and `tool_id` has no source, the component
auto-creates a Value List dropdown listing every tool currently in
`cell.tool_models` (refreshed when the set changes).

If `group` is left empty, the cell's main planning group is used (requires
`robot_cell` to be wired in). Any tool already attached to the same group is
automatically detached.

If `touch_links` is left unwired, the component defaults to the end-effector
link of the planning group (plus its parent link when the end-effector link
itself has no geometry, e.g. UR's `tool0` is a zero-geometry frame so the
parent `wrist_3_link` is included automatically). A warning is surfaced so
the auto-pick isn't silent.

A remark always announces the link the tool actually ends up attached to:
the planning group's end-effector link as returned by
`robot_cell.get_end_effector_link_name(group)`. Override that effective
frame via the `attachment_plane` input.

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

from compas_fab.ghpython import ensure_dynamic_value_list


class AttachToolToRobot(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        cell_state,
        tool_id: str,
        robot_cell,
        group: str,
        attachment_plane,
        touch_links: System.Collections.Generic.List[str],
    ):
        # Auto-populate the tool_id dropdown from the cell's tool inventory.
        # Runs even when tool_id is currently None: that's the trigger.
        if robot_cell is not None:
            ensure_dynamic_value_list(
                ghenv.Component,  # noqa: F821
                "tool_id",
                sorted(robot_cell.tool_models.keys()),
            )

        if cell_state is None or not tool_id:
            return cell_state

        if not group:
            if robot_cell is None:
                error(ghenv.Component, "`group` is empty and no `robot_cell` was wired in: provide one or the other so the tool's planning group can be resolved.")  # noqa: F821
                return cell_state
            group = robot_cell.main_group_name

        attachment_frame = None
        if attachment_plane is not None:
            attachment_frame = attachment_plane if isinstance(attachment_plane, Frame) else plane_to_compas_frame(attachment_plane)

        touch_links_list = list(touch_links) if touch_links else None

        if touch_links_list is None and robot_cell is not None:
            touch_links_list = robot_cell.default_touch_links(group)
            warning(ghenv.Component, "touch_links unwired; defaulted to {} (links the tool inevitably overlaps). Wire your own list to override.".format(touch_links_list))  # noqa: F821

        if robot_cell is not None:
            ee_link = robot_cell.get_end_effector_link_name(group.strip())
            remark(  # noqa: F821
                ghenv.Component,  # noqa: F821
                "attaching '{}' at link '{}' (group '{}' end-effector). The tool's `tool0` frame is at this link; "
                "use `attachment_plane` to rotate/offset the tool relative to it.".format(tool_id.strip(), ee_link, group.strip()),
            )

        new_state = deepcopy(cell_state)
        new_state.set_tool_attached_to_group(
            tool_id=tool_id.strip(),
            group=group.strip(),
            attachment_frame=attachment_frame,
            touch_links=touch_links_list,
        )
        return new_state
