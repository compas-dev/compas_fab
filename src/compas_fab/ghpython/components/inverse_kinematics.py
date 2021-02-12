import Grasshopper
import System
from ghpythonlib.componentbase import dotnetcompiledcomponent as component

from compas.geometry import Frame
from compas_fab.ghpython.components.icons import inverse_kinematics_icon
from compas_fab.ghpython.components import coerce_frame


class InverseKinematics(component):
    def __new__(cls):
        return Grasshopper.Kernel.GH_Component.__new__(cls,
                                                       "Inverse Kinematics",
                                                       "Inverse Kinematics",
                                                       """Calculate the robot's inverse kinematic for a given plane.""",
                                                       "COMPAS FAB",
                                                       "Planning")

    def get_ComponentGuid(self):
        return System.Guid("d396725d-8bb3-4b80-97ad-dce8061cbbe4")

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

        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "plane", "plane", "The plane to calculate the inverse kinematic for.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "start_configuration", "start_configuration",
                        "If passed, the inverse will be calculated such that the calculated joint positions \
                         differ the least from the start_configuration. Defaults to the zero configuration.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_String()
        self.SetUpParam(p, "group", "group", "The planning group used for calculation. Defaults to the robot's main planning group.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

    def RegisterOutputParams(self, pManager):
        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "a", "configuration", "The planning group's configuration.")
        self.Params.Output.Add(p)

    def SolveInstance(self, DA):
        p0 = self.marshal.GetInput(DA, 0)
        p1 = self.marshal.GetInput(DA, 1)
        p2 = self.marshal.GetInput(DA, 2)
        p3 = self.marshal.GetInput(DA, 3)
        result = self.RunScript(p0, p1, p2, p3)

        if result is not None:
            self.marshal.SetOutput(result, DA, 0, True)

    def get_Internal_Icon_24x24(self):
        return inverse_kinematics_icon

    def RunScript(self, robot, plane, start_configuration, group):
        configuration = None
        if robot and robot.client and robot.client.is_connected and plane:
            frame = coerce_frame(plane)
            configuration = robot.inverse_kinematics(frame, start_configuration, group)
        return configuration
