# r: compas_fab>=1.0.2
"""
Connect or disconnect to ROS.

COMPAS FAB v1.1.0
"""

import Grasshopper
from compas_ghpython import create_id
from scriptcontext import sticky as st

from compas_fab.backends import RosClient


class ROSConnect(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, ip, port, connect):
        ros_client = None

        ip = ip or "127.0.0.1"
        port = port or 9090

        key = create_id(ghenv.Component, "ros_client")  # noqa: F821
        ros_client = st.get(key, None)

        if ros_client:
            st[key].close()
        if connect:
            st[key] = RosClient(ip, port)
            st[key].run(5)

        ros_client = st.get(key, None)
        is_connected = ros_client.is_connected if ros_client else False
        return (ros_client, is_connected)
