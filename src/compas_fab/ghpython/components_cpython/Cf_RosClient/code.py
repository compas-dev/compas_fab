# r: compas_fab>=1.1.0
"""
Connect to a ROS bridge (ROS 1 or ROS 2).

The client is cached in sticky and survives canvas refreshes. Toggling
`connect` from True to False closes the connection. The ROS distro is
auto-detected after connecting.

COMPAS FAB v1.1.0
"""

import Grasshopper
from compas_ghpython import create_id
from scriptcontext import sticky as st

from compas_fab.backends import RosClient


class RosClientComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, host: str, port: int, connect: bool):
        host = host or "127.0.0.1"
        port = port or 9090

        key = create_id(ghenv.Component, "ros_client")  # noqa: F821
        ros_client = st.get(key, None)

        if ros_client is not None:
            try:
                ros_client.close()
            except Exception as e:
                print("Warning while closing previous ROS client: {}".format(e))
            st.pop(key, None)
            ros_client = None

        if connect:
            ros_client = RosClient(host, port)
            ros_client.run(5)
            st[key] = ros_client

        is_connected = bool(ros_client and ros_client.is_connected)
        ros_distro = None
        if is_connected:
            try:
                ros_distro = ros_client.ros_distro.value
            except Exception as e:
                print("Could not read ROS distro: {}".format(e))

        return (ros_client, is_connected, ros_distro)
