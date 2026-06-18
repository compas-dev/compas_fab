# r: compas_fab>=2.0.0
"""
Create an AnalyticalPyBulletPlanner with its own in-process PyBullet server:
closed-form analytical IK paired with PyBullet collision checking.

Inverse kinematics is solved analytically (fast, no random restarts) while
PyBullet provides in-process collision detection, so IK solutions in collision
are rejected. Supports the same fixed set of robots as the analytical solvers.

This component manages the PyBullet client for you: on first run it starts a
headless ('direct') or windowed ('gui') PyBullet server, connects, and builds the
planner. The client and planner are cached in sticky and reused across canvas
refreshes; the `robot_cell` is (re)applied via set_robot_cell on every run.

Note: the analytical kinematics convention may differ from the URDF model
(different base/flange frames). Pair this planner with a RobotCell that matches
its convention.

COMPAS FAB v2.0.0
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import create_id
from compas_ghpython import error
from scriptcontext import sticky as st

from compas_fab.backends import AnalyticalPyBulletPlanner
from compas_fab.backends import PyBulletClient
from compas_fab.backends.kinematics.solvers import PLANNER_BACKENDS
from compas_fab.ghpython import ensure_value_list

_SOLVERS = sorted(PLANNER_BACKENDS.keys())
_CONNECTION_TYPES = ["direct", "gui"]


class AnalyticalPyBulletPlannerComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, robot_cell, solver: str, connection_type: str):
        ensure_value_list(ghenv.Component, "solver", _SOLVERS, default="ur5")  # noqa: F821
        ensure_value_list(ghenv.Component, "connection_type", _CONNECTION_TYPES, default="direct")  # noqa: F821

        if robot_cell is None or not solver:
            return None

        solver = solver.strip().lower()
        if solver not in PLANNER_BACKENDS:
            error(ghenv.Component, "Unknown analytical solver '{}'. Available: {}".format(solver, ", ".join(_SOLVERS)))  # noqa: F821
            return None

        connection_type = (connection_type or "direct").strip().lower()

        key = create_id(ghenv.Component, "analytical_pybullet_planner")  # noqa: F821
        cached = st.get(key)  # (client, planner, solver, connection_type)

        # (Re)build when missing, when the solver or connection type changed, or
        # when the cached client has lost its connection.
        rebuild = cached is None or cached[2] != solver or cached[3] != connection_type
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
            planner = AnalyticalPyBulletPlanner(client, PLANNER_BACKENDS[solver]())
            cached = (client, planner, solver, connection_type)
            st[key] = cached

        planner = cached[1]
        planner.set_robot_cell(robot_cell)
        return planner
