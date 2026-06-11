# r: compas_fab>=1.1.0
"""
Create a MoveItPlanner backed by a connected RosClient.

The robot cell is taken from `ros_client.robot_cell` (set by a Load Robot Cell
From ROS component upstream), so wire that component's `ros_client` output into
here - no separate robot_cell input is needed.

The planner is cached in sticky and reused across canvas refreshes so the
planning scene is not reset on every tick. If the client disconnects and
reconnects, the cached planner is rebuilt.

COMPAS FAB v1.1.0
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import create_id
from scriptcontext import sticky as st

from compas_fab.backends import MoveItPlanner


class MoveItPlannerComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, ros_client):
        if ros_client is None or not ros_client.is_connected:
            return None

        key = create_id(ghenv.Component, "moveit_planner")  # noqa: F821
        cached = st.get(key)

        # Invalidate cache if the client has changed (e.g. user reconnected)
        if cached is not None and cached.client is not ros_client:
            cached = None

        if cached is None:
            cached = MoveItPlanner(ros_client)
            st[key] = cached

        # The cell is read from the client (set by Load Robot Cell From ROS), not
        # wired in directly. Uploading it to the planning scene here keeps planning
        # calls from re-uploading geometry.
        robot_cell = ros_client.robot_cell
        if robot_cell is not None:
            cached.set_robot_cell(robot_cell)

        return cached
