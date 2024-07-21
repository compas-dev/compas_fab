from compas_fab.backends.interfaces.client import PlannerInterface

from compas_fab.backends.pybullet.backend_features import *
from compas_fab.backends.kinematics.backend_features import *
from compas_fab.backends.kinematics.solvers import *

from compas_fab.backends.pybullet.client import PyBulletClient  # noqa: F401

__all__ = [
    "AnalyticalPyBulletPlanner",
]


class AnalyticalKinematicsPlanner(
    AnalyticalForwardKinematics,
    AnalyticalInverseKinematics,
    AnalyticalPlanCartesianMotion,
    AnalyticalSetRobotCell,
    PlannerInterface,
):
    """Analytical Inverse Kinematics Planner utilize analytical kinematics to provide fast and accurate FK, IK and Cartesian motion planning.

    This planner only support robots whose inverse kinematics is available through AnalyticalKinematics,
    such as OffsetWristKinematics and SphericalWristKinematics types.
    Note that the kinematics is not necessary equivalent to the same robot defined with URDF, the difference is in the
    base frame of the robot and the flange frames.
    For full list of implemented solvers, see the solvers module.

    The analytical nature of the kinematics solver allows for fast and accurate calculations of the forward and inverse kinematics.

    There is no collision checking available in this planner, it is also not possible to plan for collision-free IK configuration or trajectory.
    It is therefor also not possible to provide a robot cell with tools or rigid bodies.
    If collision checking is required, consider using the AnalyticalPyBulletPlanner.

    In many cases, analytical solvers are able to provide multiple solutions for the inverse kinematics problem.
    The `iter_inverse_kinematics` method will return an iterator that yields all possible solutions,
    while the `inverse_kinematics` method will return one solution at a time.

    """

    def __init__(self, kinematics_solver):
        # type: (AnalyticalKinematics) -> None
        self.kinematics_solver = kinematics_solver  # type: AnalyticalKinematics

        # Initialize all mixins
        super(AnalyticalKinematicsPlanner, self).__init__()


class AnalyticalPyBulletPlanner(
    PyBulletForwardKinematics,
    AnalyticalPybulletInverseKinematics,
    AnalyticalPlanCartesianMotion,
    PyBulletSetRobotCell,
    PyBulletSetRobotCellState,
    PlannerInterface,
):
    """Combination of PyBullet as the client for Collision Detection and Analytical Inverse Kinematics.

    This planner is based on the the PyBullet client for collision detection and use
    the analytical inverse kinematics for the calculation of the robot's inverse kinematics.

    Similar to calling other analytical inverse kinematics solvers, this planner
    supports only robots whose inverse kinematics is available through AnalyticalKinematics,
    such as OffsetWristKinematics and SphericalWristKinematics types.

    """

    def __init__(self, client, kinematics_solver):
        # type: (PyBulletClient, AnalyticalKinematics) -> None
        self._client = client  # type: PyBulletClient
        self.kinematics_solver = kinematics_solver

        # Initialize all mixins
        super(AnalyticalPyBulletPlanner, self).__init__()