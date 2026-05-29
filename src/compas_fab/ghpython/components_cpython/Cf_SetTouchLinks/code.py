# r: compas_fab>=1.1.0
"""
Set the touch_links (allowed-collision links) on a RigidBody state.

Useful when a static body is expected to touch specific robot links by
design (e.g. a floor under the base link, a fixture grazing the wrist).

Passthrough builder: the input state is copied.

COMPAS FAB v1.1.0
"""

from copy import deepcopy

import Grasshopper
import System


class SetTouchLinks(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        cell_state,
        rigid_body_id: str,
        touch_links: System.Collections.Generic.List[str],
    ):
        if cell_state is None or not rigid_body_id:
            return cell_state

        new_state = deepcopy(cell_state)
        new_state.rigid_body_states[rigid_body_id.strip()].touch_links = list(touch_links) if touch_links else []
        return new_state
