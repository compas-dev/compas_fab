"""
Load robot directly from ROS.

COMPAS FAB v1.0.2
"""

from compas.scene import SceneObject
from ghpythonlib.componentbase import executingcomponent as component
from scriptcontext import sticky as st

from compas_fab.ghpython.components import create_id


class ROSRobot(component):
    def RunScript(self, ros_client, load):
        key = create_id(self, "robot")

        if ros_client and ros_client.is_connected and load:
            # Load URDF from ROS
            robot_cell = ros_client.load_robot_cell(load_geometry=True, precision=12)
            # TODO: Need to upgrade it to a RobotCellSceneObject
            robot_cell_scene_object = SceneObject(robot_cell.robot_model)
            st[key] = (robot_cell, robot_cell_scene_object)

        # Retrieve robot from sticky
        # TODO: We should decide whether we create SceneObject here or in separate component
        robot_cell, robot_cell_scene_object = st.get(key, (None, None))
        return robot_cell, robot_cell_scene_object
