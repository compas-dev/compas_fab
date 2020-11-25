from ghpythonlib.componentbase import dotnetcompiledcomponent as component
import Grasshopper
import System

from compas_fab.robots import CollisionMesh
from compas_fab.robots import AttachedCollisionMesh
from compas_rhino.geometry import RhinoMesh
from compas_fab.ghpython.components.icons import default_icon

class AttachedCollisionMeshComponent(component):
    def __new__(cls):
        return Grasshopper.Kernel.GH_Component.__new__(cls,
                                                       "Attached Collision Mesh",
                                                       "Attached Collision Mesh",
                                                       """Add an attached collision mesh to the robot.""",
                                                       "COMPAS FAB",
                                                       "Scene")

    def get_ComponentGuid(self):
        return System.Guid("9d995079-5068-4118-9e2e-462f34154b53")

    def SetUpParam(self, p, name, nickname, description):
        p.Name = name
        p.NickName = nickname
        p.Description = description
        p.Optional = True

    def RegisterInputParams(self, pManager):
        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "scene", "scene", ":class:`compas_fab.robots.PlanningScene` The planning scene.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Mesh()
        self.SetUpParam(p, "mesh", "mesh", "A collision mesh.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_String()
        self.SetUpParam(p, "identifier", "identifier", "The identifier of the collision mesh.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_String()
        self.SetUpParam(p, "link_name", "link_name", "The robot's link name to attach the mesh to.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_String()
        self.SetUpParam(p, "touch_links", "touch_links", "Names of links that the robot is allowed to touch.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.list
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Boolean()
        self.SetUpParam(p, "add", "add", "If `True`, adds the collision mesh to the planning scene.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Boolean()
        self.SetUpParam(p, "remove", "remove", "If `True`, removes the collision mesh from the planning scene.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

    def RegisterOutputParams(self, pManager):
        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "attached_collision_mesh", "attached_collision_mesh", "A collision mesh that is attached to a robot's link.")
        self.Params.Output.Add(p)

    def SolveInstance(self, DA):
        p0 = self.marshal.GetInput(DA, 0)
        p1 = self.marshal.GetInput(DA, 1)
        p2 = self.marshal.GetInput(DA, 2)
        p3 = self.marshal.GetInput(DA, 3)
        p4 = self.marshal.GetInput(DA, 4)
        p5 = self.marshal.GetInput(DA, 5)
        p6 = self.marshal.GetInput(DA, 6)
        result = self.RunScript(p0, p1, p2, p3, p4, p5, p6)

        if result is not None:
            self.marshal.SetOutput(result, DA, 0, True)

    def get_Internal_Icon_24x24(self):
        return default_icon

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
