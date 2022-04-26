"""
Add an attached collision mesh to the robot.

COMPAS FAB v0.25.0
"""
from ghpythonlib.componentbase import executingcomponent as component

from compas_rhino.geometry import RhinoMesh
from compas_fab.robots import AttachedCollisionMesh
from compas_fab.robots import CollisionMesh


class AttachedCollisionMeshComponent(component):
    def RunScript(self, scene, mesh, identifier, link_name, touch_links, add, remove):
        attached_collision_mesh = None
        if scene and mesh and identifier and link_name:
            compas_mesh = RhinoMesh.from_geometry(mesh).to_compas()
            collision_mesh = CollisionMesh(compas_mesh, identifier)
            attached_collision_mesh = AttachedCollisionMesh(collision_mesh, link_name, touch_links)
            if add:
                scene.add_attached_collision_mesh(attached_collision_mesh)
            if remove:
                scene.remove_attached_collision_mesh(identifier)
                scene.remove_collision_mesh(identifier)
        return attached_collision_mesh
