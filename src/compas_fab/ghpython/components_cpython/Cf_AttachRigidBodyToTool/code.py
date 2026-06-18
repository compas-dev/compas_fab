# r: compas_fab>=2.0.0
"""
Attach a RigidBody to a tool inside a RobotCellState.

Use this for workpieces grasped by the tool tip. The body becomes a
"workpiece" of the tool (TargetMode.WORKPIECE resolves to its frame).

Passthrough builder: the input state is copied.

COMPAS FAB v2.0.0
"""

from copy import deepcopy

import Grasshopper
import Rhino
import System
from compas.geometry import Frame
from compas_rhino.conversions import plane_to_compas_frame


class AttachRigidBodyToTool(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, cell_state, rigid_body_id: str, tool_id: str, attachment_plane):
        if cell_state is None or not rigid_body_id or not tool_id:
            return cell_state

        attachment_frame = None
        if attachment_plane is not None:
            attachment_frame = attachment_plane if isinstance(attachment_plane, Frame) else plane_to_compas_frame(attachment_plane)

        new_state = deepcopy(cell_state)
        new_state.set_rigid_body_attached_to_tool(
            rigid_body_id=rigid_body_id.strip(),
            tool_id=tool_id.strip(),
            attachment_frame=attachment_frame,
        )
        return new_state
