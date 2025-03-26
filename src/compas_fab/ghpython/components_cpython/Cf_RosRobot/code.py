# r: compas_fab>=1.0.2
"""
Load robot directly from ROS.

COMPAS FAB v1.0.2
"""

import Grasshopper
from compas.scene import SceneObject
from compas_ghpython import create_id
from scriptcontext import sticky as st


class ROSRobot(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, ros_client, load):
        key = create_id(ghenv.Component, "robot")  # noqa: F821

        if ros_client and ros_client.is_connected and load:
            # Load URDF from ROS
            st[key] = ros_client.load_robot(load_geometry=True, precision=12)
            st[key].scene_object = SceneObject(st[key].model)

        robot = st.get(key, None)
        if robot:  # client sometimes need to be restarted, without needing to reload geometry
            robot.client = ros_client
        return robot
