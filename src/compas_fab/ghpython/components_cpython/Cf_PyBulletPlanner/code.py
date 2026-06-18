# r: compas_fab>=2.0.0
"""
Create a PyBulletPlanner with its own in-process PyBullet physics server.

This component manages the PyBullet client for you: on first run it starts a
headless ('direct') or windowed ('gui') PyBullet server, connects, and builds the
planner. The client and planner are cached in sticky and reused across canvas
refreshes; the `robot_cell` is (re)applied via set_robot_cell on every run.

Supports collision checking, forward/inverse kinematics and Cartesian motion
in-process (no ROS). Free-space `plan_motion` is not yet implemented for this
backend.

COMPAS FAB v2.0.0
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import create_id
from compas_ghpython import error
from scriptcontext import sticky as st

from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.ghpython import ensure_value_list

_CONNECTION_TYPES = ["direct", "gui"]


class PyBulletPlannerComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, robot_cell, connection_type: str):
        ensure_value_list(ghenv.Component, "connection_type", _CONNECTION_TYPES, default="direct")  # noqa: F821

        if robot_cell is None:
            return None

        connection_type = (connection_type or "direct").strip().lower()

        key = create_id(ghenv.Component, "pybullet_planner")  # noqa: F821
        cached = st.get(key)  # (client, planner, connection_type)

        # (Re)build the client + planner when missing, when the connection type
        # changed, or when the cached client has lost its connection.
        rebuild = cached is None or cached[2] != connection_type
        if not rebuild:
            try:
                rebuild = not cached[0].is_connected
            except Exception:
                rebuild = True

        if rebuild:
            if cached is not None:
                try:
                    cached[0].disconnect()
                except Exception:
                    pass
            try:
                client = PyBulletClient(connection_type=connection_type)
                client.connect()
            except Exception as e:
                st.pop(key, None)
                error(ghenv.Component, "Failed to start PyBullet ('{}'): {}".format(connection_type, e))  # noqa: F821
                return None
            cached = (client, PyBulletPlanner(client), connection_type)
            st[key] = cached

        planner = cached[1]
        planner.set_robot_cell(robot_cell)
        return planner
