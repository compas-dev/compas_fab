# r: compas_fab>=1.1.0
"""
Connect to a ROS bridge (ROS 1 or ROS 2).

The client is cached in sticky and survives canvas refreshes. Toggling
`connect` from True to False closes the connection. The ROS distro is
auto-detected after connecting.

COMPAS FAB v1.1.0
"""

import Grasshopper
import Rhino
import roslibpy
import System
from compas_ghpython import create_id
from compas_ghpython import remark
from compas_ghpython import warning
from scriptcontext import sticky as st

from compas_fab.backends import RosClient
from compas_fab.ghpython import ensure_boolean_toggle


def _roslibpy_supports_transport():
    """The ``transport`` argument was introduced in roslibpy 2.1."""
    try:
        major, minor = (int(part) for part in roslibpy.__version__.split(".")[:2])
        return (major, minor) >= (2, 1)
    except Exception:
        return False


class RosClientComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, host: str, port: int, connect: bool, transport: str):
        ensure_boolean_toggle(ghenv.Component, "connect", default=False)  # noqa: F821

        host = host or "127.0.0.1"
        port = port or 9090
        transport = transport.strip() if transport else None

        key = create_id(ghenv.Component, "ros_client")  # noqa: F821
        ros_client = st.get(key, None)

        if ros_client is not None:
            try:
                ros_client.close()
            except Exception as e:
                warning(ghenv.Component, "Error closing previous ROS client: {}".format(e))  # noqa: F821
            st.pop(key, None)
            ros_client = None

        if transport and not _roslibpy_supports_transport():
            warning(ghenv.Component, "The 'transport' input requires roslibpy >= 2.1; ignoring it.")  # noqa: F821
            transport = None

        if connect:
            if transport:
                ros_client = RosClient(host, port, transport=transport)
            else:
                ros_client = RosClient(host, port)
            ros_client.run(5)
            st[key] = ros_client

        is_connected = bool(ros_client and ros_client.is_connected)
        ros_distro = None
        if is_connected:
            try:
                ros_distro = ros_client.ros_distro.value
            except Exception as e:
                remark(ghenv.Component, "Could not detect ROS distro: {}".format(e))  # noqa: F821

        return (ros_client, is_connected, ros_distro)
