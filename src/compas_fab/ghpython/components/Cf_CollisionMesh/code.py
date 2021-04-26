import Grasshopper
import System
from ghpythonlib.componentbase import dotnetcompiledcomponent as component

from compas_rhino.geometry import RhinoMesh
from compas_fab.ghpython.components.icons import collision_mesh_icon
from compas_fab.robots import CollisionMesh


class CollisionMeshComponent(component):
    def __new__(cls):
        return Grasshopper.Kernel.GH_Component.__new__(cls,
                                                       "Collision Mesh",
                                                       "Collision Mesh",
                                                       """Add or remove a collision mesh from the planning scene.""",
                                                       "COMPAS FAB",
                                                       "Scene")

    def get_ComponentGuid(self):
        return System.Guid("fce04d23-0239-43d8-baa5-1351ccc8f0ac")

    def SetUpParam(self, p, name, nickname, description):
        p.Name = name
        p.NickName = nickname
        p.Description = description
        p.Optional = True

    def get_Internal_Icon_24x24(self):
        return collision_mesh_icon

    def RegisterInputParams(self, pManager):

        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "scene", "scene", "The planning scene.")
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

        p = Grasshopper.Kernel.Parameters.Param_Boolean()
        self.SetUpParam(p, "add", "add", "If `True`, adds the collision mesh to the planning scene.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Boolean()
        self.SetUpParam(p, "remove", "remove", "If `True`, removes the collision mesh from the planning scene.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

    def RegisterOutputParams(self, pManager):
        p = Grasshopper.Kernel.Parameters.Param_Boolean()
        self.SetUpParam(p, "ok", "ok", "`True` if the operation was sucessfully performed.")
        self.Params.Output.Add(p)

    def SolveInstance(self, DA):
        p0 = self.marshal.GetInput(DA, 0)
        p1 = self.marshal.GetInput(DA, 1)
        p2 = self.marshal.GetInput(DA, 2)
        p3 = self.marshal.GetInput(DA, 3)
        p4 = self.marshal.GetInput(DA, 4)
        result = self.RunScript(p0, p1, p2, p3, p4)

        if result is not None:
            self.marshal.SetOutput(result, DA, 0, True)

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
