import logging
from compas_fab.backends import PyBulletClient
from compas_fab.backends.pybullet import LOG
from compas_fab.backends.kinematics import AnalyticalInverseKinematics
from compas_fab.backends.kinematics import AnalyticalPlanCartesianMotion

LOG.setLevel(logging.ERROR)


class AnalyticalPyBulletClient(PyBulletClient):
    def inverse_kinematics(self, *args, **kwargs):
        return AnalyticalInverseKinematics(self)(*args, **kwargs)

    def plan_cartesian_motion(self, *args, **kwargs):
        return AnalyticalPlanCartesianMotion(self)(*args, **kwargs)
