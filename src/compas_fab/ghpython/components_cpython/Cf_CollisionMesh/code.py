# r: compas_fab>=1.0.2
"""
Add or remove a collision mesh from the planning scene.

COMPAS FAB v1.1.0
"""

import Grasshopper
import Rhino
from compas_rhino.conversions import mesh_to_compas

from compas_fab.ghpython.components import error
from compas_fab.ghpython.components import message
from compas_fab.robots import CollisionMesh


class CollisionMeshComponent(Grasshopper.Kernel.GH_ScriptInstance):
    @property
    def component(self):
        return ghenv.Component

    def RunScript(self, scene, mesh: Rhino.Geometry.Mesh, identifier: str, add: bool, append: bool, remove: bool):
        ok = False
        message(self.component, "")

        if (add and append) or (append and remove) or (add and remove):
            error(self.component, "Use only one operation at a time\n(add, append or remove)")
            return

        if scene and mesh and identifier:
            compas_mesh = mesh_to_compas(mesh)
            collision_mesh = CollisionMesh(compas_mesh, identifier)
            if add:
                scene.add_collision_mesh(collision_mesh)
                message(self.component, "Added")
                ok = True
            if append:
                scene.append_collision_mesh(collision_mesh)
                message(self.component, "Appended")
                ok = True
            if remove:
                scene.remove_collision_mesh(identifier)
                message(self.component, "Removed")
                ok = True
        return ok
