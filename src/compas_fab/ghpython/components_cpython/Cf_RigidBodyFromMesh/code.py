# r: compas_fab>=1.1.0
"""
Construct a RigidBody from a Rhino mesh (and optional separate collision mesh).

If only `visual_mesh` is wired, the same mesh is used for collision checking.
Use `native_scale` to declare a non-meter document scale (e.g. 0.001 for mm).

COMPAS FAB v1.1.0
"""

import Grasshopper
import Rhino
import System
from compas_rhino.conversions import mesh_to_compas

from compas_fab.robots import RigidBody


class RigidBodyFromMesh(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, visual_mesh, collision_mesh, native_scale: float):
        if visual_mesh is None:
            return None

        native_scale = native_scale if native_scale else 1.0
        c_visual = mesh_to_compas(visual_mesh)
        c_collision = mesh_to_compas(collision_mesh) if collision_mesh else c_visual

        return RigidBody(
            visual_meshes=[c_visual],
            collision_meshes=[c_collision],
            native_scale=native_scale,
        )
