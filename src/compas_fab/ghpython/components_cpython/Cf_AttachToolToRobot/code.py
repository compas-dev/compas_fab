# r: compas_fab>=1.1.0
"""
Attach a tool to a robot planning group inside a RobotCellState.

Passthrough builder: the input state is copied, the attachment is set, and
the new state is returned.

If `group` is left empty, the cell's main planning group is used (requires
`robot_cell` to be wired in). Any tool already attached to the same group is
automatically detached.

If `touch_links` is left unwired, the component defaults to the end-effector
link of the planning group (plus its parent link when the end-effector link
itself has no geometry, e.g. UR's `tool0` is a zero-geometry frame so the
parent `wrist_3_link` is included automatically). A warning is surfaced so
the auto-pick isn't silent.

COMPAS FAB v1.1.0
"""

from copy import deepcopy

import Grasshopper
import System
from compas.geometry import Frame
from compas_ghpython import error
from compas_ghpython import warning
from compas_rhino.conversions import plane_to_compas_frame


def _default_touch_links(robot_cell, group):
    """End-effector link of the group, plus its parent link if the EE link
    has no geometry (covers the common UR `tool0` frame-link case)."""
    ee_name = robot_cell.get_end_effector_link_name(group)
    links = [ee_name]
    ee_link = robot_cell.robot_model.get_link_by_name(ee_name)
    if ee_link is not None and not (ee_link.visual or ee_link.collision):
        parent_joint = robot_cell.robot_model.find_parent_joint(ee_link)
        if parent_joint is not None:
            links.append(parent_joint.parent.link)
    return links


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
        if cell_state is None or not tool_id:
            return cell_state

        if not group:
            if robot_cell is None:
                error(ghenv.Component, "`group` is empty and no `robot_cell` was wired in — provide one or the other so the tool's planning group can be resolved.")  # noqa: F821
                return cell_state
            group = robot_cell.main_group_name

        attachment_frame = None
        if attachment_plane is not None:
            attachment_frame = attachment_plane if isinstance(attachment_plane, Frame) else plane_to_compas_frame(attachment_plane)

        touch_links_list = list(touch_links) if touch_links else None

        if touch_links_list is None and robot_cell is not None:
            touch_links_list = _default_touch_links(robot_cell, group)
            warning(ghenv.Component, "touch_links unwired; defaulted to {} (links the tool inevitably overlaps). Wire your own list to override.".format(touch_links_list))  # noqa: F821

        new_state = deepcopy(cell_state)
        new_state.set_tool_attached_to_group(
            tool_id=tool_id.strip(),
            group=group.strip(),
            attachment_frame=attachment_frame,
            touch_links=touch_links_list,
        )
        return new_state
