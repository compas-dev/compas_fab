import Grasshopper
import System
from compas.geometry import Frame
from ghpythonlib.componentbase import dotnetcompiledcomponent as component
from scriptcontext import sticky as st

from compas_fab.ghpython.components import create_id
from compas_fab.ghpython.components.icons import default_icon


class PlanCartesianMotion(component):
    def __new__(cls):
        return Grasshopper.Kernel.GH_Component.__new__(cls,
                                                       "Plan Cartesian Motion",
                                                       "Plan Cartesian Motion",
                                                       """Calculate a cartesian motion path (linear in tool space).""",
                                                       "COMPAS FAB",
                                                       "Planning")

    def get_ComponentGuid(self):
        return System.Guid("58cdd2ab-8b83-4a35-8689-f80a43a6d667")

    def SetUpParam(self, p, name, nickname, description):
        p.Name = name
        p.NickName = nickname
        p.Description = description
        p.Optional = True

    def RegisterInputParams(self, pManager):
        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "robot", "robot", "The robot.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Plane()
        self.SetUpParam(p, "planes", "planes", "The planes through which the path is defined.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.list
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "start_configuration", "start_configuration",
                        "The robot's full configuration, i.e. values for all configurable joints of the entire robot, at the starting position. Defaults to the all-zero configuration.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_String()
        self.SetUpParam(p, "group", "group", "The planning group used for calculation. Defaults to the robot's main planning group.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "attached_collision_meshes", "attached_collision_meshes", "A list of attached collision meshes to be included for planning.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.list
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "path_constraints", "path_constraints",
                        "Optional constraints that can be imposed along the solution path. Note that path calculation won't work if the start_configuration violates these constraints. Defaults to None.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.list
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Number()
        self.SetUpParam(p, "max_step", "max_step", "The approximate distance between the calculated points. (Defined in the robot's units.) Defaults to 0.01.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Boolean()
        self.SetUpParam(p, "compute", "compute", "If `True`, calculates a trajectory.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

    def RegisterOutputParams(self, pManager):
        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "trajectory", "trajectory", "The calculated trajectory.")
        self.Params.Output.Add(p)

    def SolveInstance(self, DA):
        p0 = self.marshal.GetInput(DA, 0)
        p1 = self.marshal.GetInput(DA, 1)
        p2 = self.marshal.GetInput(DA, 2)
        p3 = self.marshal.GetInput(DA, 3)
        p4 = self.marshal.GetInput(DA, 4)
        p5 = self.marshal.GetInput(DA, 5)
        p6 = self.marshal.GetInput(DA, 6)
        p7 = self.marshal.GetInput(DA, 7)
        result = self.RunScript(p0, p1, p2, p3, p4, p5, p6, p7)

        if result is not None:
            self.marshal.SetOutput(result, DA, 0, True)

    def get_Internal_Icon_24x24(self):
        return default_icon

    def RunScript(self, robot, planes, start_configuration, group, attached_collision_meshes, path_constraints, max_step, compute):

        key = create_id(self, 'trajectory')

        max_step = float(max_step) if max_step else 0.01
        path_constraints = list(path_constraints) if path_constraints else None
        attached_collision_meshes = list(attached_collision_meshes) if attached_collision_meshes else None

        if robot and robot.client and robot.client.is_connected and start_configuration and planes and compute:
            frames = [Frame(plane.Origin, plane.XAxis, plane.YAxis) for plane in planes]
            st[key] = robot.plan_cartesian_motion(frames,
                                                  start_configuration=start_configuration,
                                                  group=group,
                                                  options=dict(
                                                      max_step=max_step,
                                                      path_constraints=path_constraints,
                                                      attached_collision_meshes=attached_collision_meshes
                                                  ))

        trajectory = st.get(key, None)
        return trajectory
