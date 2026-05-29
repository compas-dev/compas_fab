# r: compas_fab>=1.1.0
"""
Attach a RigidBody to a specific robot link inside a RobotCellState.

Use this for items rigidly mounted on the robot (cable dress-packs,
cameras on a wrist, etc.) - things that move with a link but aren't a
tool TCF.

Passthrough builder: the input state is copied.

COMPAS FAB v1.1.0
"""

from copy import deepcopy

import Grasshopper
import System
from compas.geometry import Frame
from compas_rhino.conversions import plane_to_compas_frame


class AttachRigidBodyToLink(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        cell_state,
        rigid_body_id: str,
        link_name: str,
        attachment_plane,
        touch_links: System.Collections.Generic.List[str],
    ):
        if cell_state is None or not rigid_body_id or not link_name:
            return cell_state

        attachment_frame = None
        if attachment_plane is not None:
            attachment_frame = attachment_plane if isinstance(attachment_plane, Frame) else plane_to_compas_frame(attachment_plane)

        touch_links_list = list(touch_links) if touch_links else None

        new_state = deepcopy(cell_state)
        new_state.set_rigid_body_attached_to_link(
            rigid_body_id=rigid_body_id.strip(),
            link_name=link_name.strip(),
            attachment_frame=attachment_frame,
            touch_links=touch_links_list,
        )
        return new_state
