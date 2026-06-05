# r: compas_fab>=1.1.0
"""
Construct a RigidBody from a Rhino mesh (and optional separate collision mesh).

If only `visual_mesh` is wired, the same mesh is used for collision checking.
Use `native_scale` to declare a non-meter document scale (e.g. 0.001 for mm).

COMPAS FAB v1.1.0
"""

import Grasshopper
import Rhino
import rhinoscriptsyntax as rs
import System
from compas_ghpython import warning as gh_warning
from compas_rhino.conversions import mesh_to_compas
from compas_fab.robots import RigidBody


class RigidBodyFromMesh(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, visual_mesh, collision_mesh, native_scale: float):
        if visual_mesh is None:
            return None

        native_scale = native_scale if native_scale else 1.0
        c_visual = mesh_to_compas(rs.coercemesh(visual_mesh)) if visual_mesh else None
        c_collision = mesh_to_compas(rs.coercemesh(collision_mesh)) if collision_mesh else None
        if collision_mesh is None:
            gh_warning(ghenv.Component, "No collision mesh provided: the rigid body will be visualized but ignored for collision checking.")  # noqa: F821

        return RigidBody(
            visual_meshes=[c_visual] if c_visual else [],
            collision_meshes=[c_collision] if c_collision else [],
            native_scale=native_scale,
        )
