# r: compas_fab>=1.1.0
"""
Construct a ToolModel from a Rhino mesh and a TCP plane.

`tcp_plane` may be a Rhino Plane or a compas Frame; it is the Tool
Coordinate Frame expressed in the robot's `tool0` link frame. If
`collision_mesh` is unwired, `visual_mesh` is reused for collision
checks.

The result is a static (no joints) `ToolModel` ready to feed into
`AddToolToCell` or `AddAndAttachTool`. Kinematic tools (with their own
joints) should be loaded from URDF instead.

COMPAS FAB v1.1.0
"""

import Grasshopper
import Rhino
import System
from compas.geometry import Frame
from compas_rhino.conversions import mesh_to_compas
from compas_rhino.conversions import plane_to_compas_frame
from compas_robots import ToolModel


class ToolFromMesh(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, visual_mesh, tcp_plane, collision_mesh, name: str):
        if visual_mesh is None or tcp_plane is None:
            return None

        frame = tcp_plane if isinstance(tcp_plane, Frame) else plane_to_compas_frame(tcp_plane)
        c_visual = mesh_to_compas(visual_mesh)
        c_collision = mesh_to_compas(collision_mesh) if collision_mesh else c_visual

        return ToolModel(
            visual=c_visual,
            frame_in_tool0_frame=frame,
            collision=c_collision,
            name=(name or "attached_tool").strip(),
        )
