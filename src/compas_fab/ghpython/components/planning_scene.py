import Grasshopper
import System
from ghpythonlib.componentbase import dotnetcompiledcomponent as component
from scriptcontext import sticky as st

from compas_fab.ghpython.components import create_id
from compas_fab.ghpython.components.icons import default_icon
from compas_fab.robots import PlanningScene


class PlanningSceneComponent(component):
    def __new__(cls):
        return Grasshopper.Kernel.GH_Component.__new__(cls,
                                                       "Planning Scene",
                                                       "Planning Scene",
                                                       """Create a planning scene.""",
                                                       "COMPAS FAB",
                                                       "Scene")

    def get_ComponentGuid(self):
        return System.Guid("cbdef67b-a5c2-453e-8e73-4488bd23fc49")

    def SetUpParam(self, p, name, nickname, description):
        p.Name = name
        p.NickName = nickname
        p.Description = description
        p.Optional = True

    def get_Internal_Icon_24x24(self):
        return default_icon

    def RegisterInputParams(self, pManager):
        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "robot", "robot", "The robot.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

    def RegisterOutputParams(self, pManager):
        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "scene", "scene", "The planning scene.")
        self.Params.Output.Add(p)

    def SolveInstance(self, DA):
        p0 = self.marshal.GetInput(DA, 0)
        result = self.RunScript(p0)

        if result is not None:
            self.marshal.SetOutput(result, DA, 0, True)

    def RunScript(self, robot):
        key = create_id(self, 'planning_scene')
        if robot:
            st[key] = PlanningScene(robot)
        return st.get(key, None)
