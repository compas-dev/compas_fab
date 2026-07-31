# r: compas_fab>=2.0.1
"""
Construct a ToolModel from a Rhino mesh and a TCP plane.

`tcp_plane` may be a Rhino Plane or a compas Frame; it is the Tool
Coordinate Frame, expressed in the same coordinates you modelled the mesh
in. If `collision_mesh` is unwired, `visual_mesh` is reused for collision
checks.

`base_plane` is where the robot's flange takes hold of the geometry, also
in the coordinates you modelled in. Its Z axis points away from the robot,
matching the planning groups of the robots in the library, so a tool drawn
reaching along world Z needs no `base_plane` at all. Wire one when the tool
was drawn along another axis: for a tool reaching along world X, that is
the plane with Z = world X. Nothing is baked into the mesh, so the plane
can be re-wired at any time.

A remark is surfaced when the TCP does not sit roughly on the tool's +Z,
since that means the tool will point sideways once attached.

The result is a static (no joints) `ToolModel` ready to feed into a
`Load Robot Cell` component's `tools` input. Kinematic tools (with their
own joints) should be loaded from URDF instead.

COMPAS FAB v2.0.1
"""

from math import degrees

import Grasshopper
import Rhino
import rhinoscriptsyntax as rs
import System
from compas.geometry import Frame
from compas.geometry import Vector
from compas_ghpython import remark
from compas_ghpython import warning as gh_warning
from compas_rhino.conversions import mesh_to_compas
from compas_rhino.conversions import plane_to_compas_frame
from compas_robots import ToolModel

# Beyond this angle between the tool's +Z and its TCP, the tool is considered
# to be pointing somewhere other than away from the flange.
MAX_TCP_OFF_AXIS_ANGLE = 30.0


def _off_axis_remark(tool):
    """Text for the mounting-axis remark, or None when the tool looks fine.

    The TCP direction is only a hint: it says nothing about the roll of the tool
    and is meaningless for a TCP sitting on the mount itself, so it is reported
    rather than applied.
    """
    tcp_direction = Vector(*tool.frame.point)
    if tcp_direction.length < 1e-6:
        return None

    angle = degrees(tcp_direction.angle(Vector.Zaxis()))
    if angle < MAX_TCP_OFF_AXIS_ANGLE:
        return None

    message = ("TCP sits {:.0f} deg off the tool's +Z axis, so this tool will point sideways once attached: tools mount with +Z pointing away from the flange. ").format(angle)

    unitized = tcp_direction.unitized()
    if abs(unitized.x) > 0.99:
        message += "Your tool reaches along X, so set `base_plane` to a plane with its Z axis along world X (origin, x=(0,1,0), y=(0,0,1))."
    elif abs(unitized.y) > 0.99:
        message += "Your tool reaches along Y, so set `base_plane` to a plane with its Z axis along world Y (origin, x=(0,0,1), y=(1,0,0))."
    else:
        message += "Set `base_plane` to a plane whose Z axis runs along your tool, with its X axis picked to give the roll you want."
    return message


class ToolFromMesh(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, name: str, visual_mesh, collision_mesh, tcp_plane, base_plane):
        if visual_mesh is None or tcp_plane is None:
            return None

        frame = tcp_plane if isinstance(tcp_plane, Frame) else plane_to_compas_frame(tcp_plane)
        base_frame = None
        if base_plane is not None:
            base_frame = base_plane if isinstance(base_plane, Frame) else plane_to_compas_frame(base_plane)
        c_visual = mesh_to_compas(rs.coercemesh(visual_mesh)) if visual_mesh else None
        c_collision = mesh_to_compas(rs.coercemesh(collision_mesh)) if collision_mesh else None

        if collision_mesh is None:
            gh_warning(ghenv.Component, "No collision mesh provided: the tool will be visualized but ignored for collision checking.")  # noqa: F821

        # `base_frame` is only passed when wired, so an older compas_robots keeps
        # working for definitions that don't use it.
        kwargs = {"base_frame": base_frame} if base_frame else {}
        tool = ToolModel(
            visual=c_visual,
            frame_in_tool0_frame=frame,
            collision=c_collision,
            name=(name or "attached_tool").strip(),
            **kwargs,
        )

        message = _off_axis_remark(tool)
        if message:
            remark(ghenv.Component, message)  # noqa: F821

        return tool
