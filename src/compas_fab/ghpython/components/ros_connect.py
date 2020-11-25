import Grasshopper
import System
from ghpythonlib.componentbase import dotnetcompiledcomponent as component
from scriptcontext import sticky as st

from compas_fab.backends import RosClient
from compas_fab.ghpython.components import create_id
from compas_fab.ghpython.components.icons import ros_connect_icon


class ROSConnect(component):
    def __new__(cls):
        instance = Grasshopper.Kernel.GH_Component.__new__(cls,
                                                           "ROS Connect",
                                                           "ROS Connect",
                                                           """Connect or disconnect to ROS""",
                                                           "COMPAS FAB",
                                                           "ROS")
        return instance

    def get_ComponentGuid(self):
        return System.Guid("cdd47086-f902-4b77-825b-6b79c3aaecc1")

    def SetUpParam(self, p, name, nickname, description):
        p.Name = name
        p.NickName = nickname
        p.Description = description
        p.Optional = True

    def RegisterInputParams(self, pManager):

        p = Grasshopper.Kernel.Parameters.Param_String()
        self.SetUpParam(p, "ip", "ip", "The ip address of ROS master. Defaults to 127.0.0.1")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Integer()
        self.SetUpParam(p, "port", "port", "The port of ROS master. Defaults to 9090.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Boolean()
        self.SetUpParam(p, "connect", "connect", "If `True`, connect to ROS. Defaults to False.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Boolean()
        self.SetUpParam(p, "disconnect", "disconnect", "If `True`, disconnect from ROS. Defaults to False.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

    def RegisterOutputParams(self, pManager):
        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "ros_client", "ros_client", "The ROS client.")
        self.Params.Output.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Boolean()
        self.SetUpParam(p, "is_connected", "is_connected", "`True` if connection established.")
        self.Params.Output.Add(p)

    def SolveInstance(self, DA):
        p0 = self.marshal.GetInput(DA, 0)
        p1 = self.marshal.GetInput(DA, 1)
        p2 = self.marshal.GetInput(DA, 2)
        p3 = self.marshal.GetInput(DA, 3)
        result = self.RunScript(p0, p1, p2, p3)

        if result is not None:
            if not hasattr(result, '__getitem__'):
                self.marshal.SetOutput(result, DA, 0, True)
            else:
                self.marshal.SetOutput(result[0], DA, 0, True)
                self.marshal.SetOutput(result[1], DA, 1, True)

    def get_Internal_Icon_24x24(self):
        return ros_connect_icon

    def RunScript(self, ip, port, connect, disconnect):
        ros_client = None

        ip = ip or '127.0.0.1'
        port = port or 9090

        key = create_id(self, 'ros_client')
        ros_client = st.get(key, None)

        if ros_client and (connect or disconnect):
            ros_client.close()

        if connect:
            st[key] = RosClient(ip, 9090)
            st[key].run(5)

        ros_client = st.get(key, None)
        is_connected = ros_client.is_connected if ros_client else False
        return (ros_client, is_connected)
