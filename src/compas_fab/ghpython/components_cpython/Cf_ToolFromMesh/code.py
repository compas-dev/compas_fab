# r: compas_fab>=2.0.0
"""
Construct a ToolModel from a Rhino mesh and a TCP plane.

`tcp_plane` may be a Rhino Plane or a compas Frame; it is the Tool
Coordinate Frame expressed in the robot's `tool0` link frame. If
`collision_mesh` is unwired, `visual_mesh` is reused for collision
checks.

The result is a static (no joints) `ToolModel` ready to feed into a
`Load Robot Cell` component's `tools` input. Kinematic tools (with their
own joints) should be loaded from URDF instead.

COMPAS FAB v2.0.0
"""

import Grasshopper
import Rhino
import rhinoscriptsyntax as rs
import System
from compas.geometry import Frame
from compas_ghpython import warning as gh_warning
from compas_rhino.conversions import mesh_to_compas
from compas_rhino.conversions import plane_to_compas_frame
from compas_robots import ToolModel


class ToolFromMesh(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, name: str, visual_mesh, collision_mesh, tcp_plane):
        if visual_mesh is None or tcp_plane is None:
            return None

        frame = tcp_plane if isinstance(tcp_plane, Frame) else plane_to_compas_frame(tcp_plane)
        c_visual = mesh_to_compas(rs.coercemesh(visual_mesh)) if visual_mesh else None
        c_collision = mesh_to_compas(rs.coercemesh(collision_mesh)) if collision_mesh else None

        if collision_mesh is None:
            gh_warning(ghenv.Component, "No collision mesh provided: the tool will be visualized but ignored for collision checking.")  # noqa: F821

        return ToolModel(
            visual=c_visual,
            frame_in_tool0_frame=frame,
            collision=c_collision,
            name=(name or "attached_tool").strip(),
        )
