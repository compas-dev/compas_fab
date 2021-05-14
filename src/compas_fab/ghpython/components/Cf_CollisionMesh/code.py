"""
Add or remove a collision mesh from the planning scene.

COMPAS FAB v0.18.1
"""
from compas_rhino.geometry import RhinoMesh
from ghpythonlib.componentbase import executingcomponent as component

from compas_fab.robots import CollisionMesh


class CollisionMeshComponent(component):
    def RunScript(self, scene, M, name, add, remove):
        ok = False
        if scene and M and name:
            mesh = RhinoMesh.from_geometry(M).to_compas()
            collision_mesh = CollisionMesh(mesh, name)
            if add:
                scene.add_collision_mesh(collision_mesh)
                ok = True
            if remove:
                scene.remove_collision_mesh(name)
                ok = True
        return ok
