"""
Add an attached collision mesh to the robot.

COMPAS FAB v1.0.2
"""

from compas_rhino.conversions import mesh_to_compas
from ghpythonlib.componentbase import executingcomponent as component

from compas_fab.robots import AttachedCollisionMesh
from compas_fab.robots import CollisionMesh


class AttachedCollisionMeshComponent(component):
    def RunScript(self, scene, mesh, identifier, link_name, touch_links, add, remove):
        attached_collision_mesh = None
        if scene and mesh and identifier and link_name:
            compas_mesh = mesh_to_compas(mesh)
            collision_mesh = CollisionMesh(compas_mesh, identifier)
            attached_collision_mesh = AttachedCollisionMesh(collision_mesh, link_name, touch_links)
            if add:
                scene.add_attached_collision_mesh(attached_collision_mesh)
            if remove:
                scene.remove_attached_collision_mesh(identifier)
                scene.remove_collision_mesh(identifier)
        return attached_collision_mesh
