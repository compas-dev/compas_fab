"""
Load robot directly from ROS.

COMPAS FAB v1.1.1
"""

from compas_ghpython import create_id
from ghpythonlib.componentbase import executingcomponent as component
from scriptcontext import sticky as st

from compas.scene import SceneObject


class ROSRobot(component):
    def RunScript(self, ros_client, load):
        key = create_id(self, "robot")

        if ros_client and ros_client.is_connected and load:
            # Load URDF from ROS
            st[key] = ros_client.load_robot(load_geometry=True, precision=12)
            st[key].scene_object = SceneObject(item=st[key].model)

        robot = st.get(key, None)
        if robot:  # client sometimes need to be restarted, without needing to reload geometry
            robot.client = ros_client
        return robot
