"""
Add or remove a collision mesh from the planning scene.

COMPAS FAB v1.0.1
"""

from compas_rhino.conversions import mesh_to_compas
from ghpythonlib.componentbase import executingcomponent as component

from compas_fab.robots import CollisionMesh


class CollisionMeshComponent(component):
    def RunScript(self, scene, M, name, add, append, remove):
        ok = False
        self.Message = ""

        if (add and append) or (append and remove) or (add and remove):
            self.Message = "Use only one operation at a time\n(add, append or remove)"
            raise Exception(self.Message)

        if scene and M and name:
            mesh = mesh_to_compas(M)
            collision_mesh = CollisionMesh(mesh, name)
            if add:
                scene.add_collision_mesh(collision_mesh)
                self.Message = "Added"
                ok = True
            if append:
                scene.append_collision_mesh(collision_mesh)
                self.Message = "Appended"
                ok = True
            if remove:
                scene.remove_collision_mesh(name)
                self.Message = "Removed"
                ok = True
        return ok
