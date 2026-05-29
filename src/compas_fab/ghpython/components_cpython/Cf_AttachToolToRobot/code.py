# r: compas_fab>=1.1.0
"""
Attach a tool to a robot planning group inside a RobotCellState.

Passthrough builder: the input state is copied, the attachment is set, and
the new state is returned.

Any tool already attached to the same group is automatically detached.

COMPAS FAB v1.1.0
"""

from copy import deepcopy

import Grasshopper
import System
from compas.geometry import Frame
from compas_rhino.conversions import plane_to_compas_frame


class AttachToolToRobot(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        cell_state,
        tool_id: str,
        group: str,
        attachment_plane,
        touch_links: System.Collections.Generic.List[str],
    ):
        if cell_state is None or not tool_id or not group:
            return cell_state

        attachment_frame = None
        if attachment_plane is not None:
            attachment_frame = attachment_plane if isinstance(attachment_plane, Frame) else plane_to_compas_frame(attachment_plane)

        touch_links_list = list(touch_links) if touch_links else None

        new_state = deepcopy(cell_state)
        new_state.set_tool_attached_to_group(
            tool_id=tool_id.strip(),
            group=group.strip(),
            attachment_frame=attachment_frame,
            touch_links=touch_links_list,
        )
        return new_state
