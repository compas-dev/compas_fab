"""
Connect or disconnect to ROS.

COMPAS FAB v0.19.1
"""
from ghpythonlib.componentbase import executingcomponent as component
from scriptcontext import sticky as st

from compas_fab.backends import RosClient
from compas_fab.ghpython.components import create_id


class ROSConnect(component):
    def RunScript(self, ip, port, connect):
        ros_client = None

        ip = ip or '127.0.0.1'
        port = port or 9090

        key = create_id(self, 'ros_client')
        ros_client = st.get(key, None)

        if ros_client:
            st[key].close()
        if connect:
            st[key] = RosClient(ip, port)
            st[key].run(5)

        ros_client = st.get(key, None)
        is_connected = ros_client.is_connected if ros_client else False
        return (ros_client, is_connected)
