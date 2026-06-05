# r: compas_fab>=1.1.0
"""
Place an unattached RigidBody at a frame in the world inside a RobotCellState.

Use for static objects in the scene (floors, tables, obstacles). For
items that move with the robot, use AttachRigidBodyToLink or
AttachRigidBodyToTool instead.

Passthrough builder: the input state is copied.

COMPAS FAB v1.1.0
"""

from copy import deepcopy

import Grasshopper
import Rhino
import System
from compas.geometry import Frame
from compas_rhino.conversions import plane_to_compas_frame


class SetRigidBodyFrame(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, cell_state, rigid_body_id: str, plane):
        if cell_state is None or not rigid_body_id or plane is None:
            return cell_state

        frame = plane if isinstance(plane, Frame) else plane_to_compas_frame(plane)

        new_state = deepcopy(cell_state)
        body_state = new_state.rigid_body_states[rigid_body_id.strip()]
        body_state.attached_to_link = None
        body_state.attached_to_tool = None
        body_state.frame = frame
        return new_state
