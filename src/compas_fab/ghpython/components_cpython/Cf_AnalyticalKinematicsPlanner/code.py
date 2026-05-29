# r: compas_fab>=1.1.0
"""
Create an AnalyticalKinematicsPlanner for closed-form FK/IK.

The planner runs in-process inside Rhino (no ROS/PyBullet required). It supports
a fixed set of robots whose kinematics are implemented analytically.

Note: the analytical kinematics convention may differ from the URDF model
(different base/flange frames). Pair this planner with a RobotCell that
matches its convention, or use it for FK/IK without visualization.

COMPAS FAB v1.1.0
"""

import Grasshopper
from compas_ghpython import create_id
from scriptcontext import sticky as st

from compas_fab.backends import AnalyticalKinematicsPlanner
from compas_fab.backends.kinematics.solvers import PLANNER_BACKENDS


class AnalyticalKinematicsPlannerComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, solver: str, robot_cell, verbose: bool):
        if not solver:
            return None

        solver = solver.strip().lower()
        if solver not in PLANNER_BACKENDS:
            raise ValueError("Unknown analytical solver '{}'. Available: {}".format(solver, ", ".join(sorted(PLANNER_BACKENDS.keys()))))

        verbose = bool(verbose)

        key = create_id(ghenv.Component, "planner_{}_{}".format(solver, verbose))  # noqa: F821
        cached = st.get(key)

        if cached is None:
            solver_instance = PLANNER_BACKENDS[solver]()
            planner = AnalyticalKinematicsPlanner(solver_instance, verbose=verbose)
            st[key] = planner
            cached = planner

        planner = cached

        if robot_cell is not None:
            planner.set_robot_cell(robot_cell)

        return planner
