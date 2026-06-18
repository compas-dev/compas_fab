# r: compas_fab>=1.1.0
"""
Attach a RigidBody to a specific robot link inside a RobotCellState.

Use this for items rigidly mounted on the robot (cable dress-packs,
cameras on a wrist, etc.) - things that move with a link but aren't a
tool TCF.

Passthrough builder: the input state is copied.

If `touch_links` is left unwired, defaults to `[link_name]` (the link the
body is attached to, which it inevitably overlaps). A warning is surfaced
so the auto-pick isn't silent.

COMPAS FAB v1.1.0
"""

from copy import deepcopy

import Grasshopper
import Rhino
import System
from compas.geometry import Frame
from compas_ghpython import warning
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

        if touch_links_list is None:
            touch_links_list = [link_name.strip()]
            warning(ghenv.Component, "touch_links unwired; defaulted to {} (the link the body is attached to). Wire your own list to override.".format(touch_links_list))  # noqa: F821

        new_state = deepcopy(cell_state)
        new_state.set_rigid_body_attached_to_link(
            rigid_body_id=rigid_body_id.strip(),
            link_name=link_name.strip(),
            attachment_frame=attachment_frame,
            touch_links=touch_links_list,
        )
        return new_state
