# -*- coding: utf-8 -*-

import math

from compas.geometry import Frame
from compas.geometry import Point

from .analytical_kinematics import AnalyticalKinematics
from .spherical_wrist import forward_kinematics_spherical_wrist
from .spherical_wrist import inverse_kinematics_spherical_wrist


class SphericalWristKinematics(AnalyticalKinematics):
    """ """

    def __init__(self, points):
        self.points = points
        super(SphericalWristKinematics, self).__init__()

    def forward(self, joint_values: list[float]) -> Frame:
        joint_values = self._pre_process(joint_values)
        return forward_kinematics_spherical_wrist(joint_values, self.points)

    def inverse(self, frame_rcf: Frame) -> list[list[float]]:
        solutions = inverse_kinematics_spherical_wrist(frame_rcf, self.points)
        return list(self._post_process(solutions))

    def _post_process(self, solutions: list[list[float]]) -> list[list[float]]:
        return solutions

    def _pre_process(self, joint_values: list[float]) -> list[float]:
        return joint_values


class Staubli_TX260LKinematics(SphericalWristKinematics):
    """Analytical IK solver for the Stäubli TX2 60L robot."""

    def __init__(self):
        points = [
            Point(0.000, 0.000, 0.375),
            Point(0.000, 0.020, 0.775),
            Point(0.450, 0.020, 0.775),
            Point(0.520, 0.020, 0.775),
        ]
        super(Staubli_TX260LKinematics, self).__init__(points)

    def _pre_process(self, joint_values: list[float]) -> list[float]:
        q1, q2, q3, q4, q5, q6 = joint_values
        q1 = -1 * q1
        q2 = q2 - math.pi / 2
        q4 = q4 * -1
        q6 = q6 * -1 + math.pi / 2
        return [q1, q2, q3, q4, q5, q6]

    def _post_process(self, solutions: list[list[float]]) -> list[list[float]]:
        for q1, q2, q3, q4, q5, q6 in solutions:
            q1 = -1 * q1
            q2 = q2 + math.pi / 2
            q4 = q4 * -1
            q6 = q6 * -1 + math.pi / 2
            yield [q1, q2, q3, q4, q5, q6]


class ABB_IRB4600_40_255Kinematics(SphericalWristKinematics):
    """Analytical IK solver for the ABB IRB4600 40/255 robot."""

    def __init__(self):
        points = [
            Point(0.175, 0.000, 0.495),
            Point(0.175, 0.000, 1.590),
            Point(1.446, 0.000, 1.765),
            Point(1.581, 0.000, 1.765),
        ]
        super(ABB_IRB4600_40_255Kinematics, self).__init__(points)

    def _pre_process(self, joint_values: list[float]) -> list[float]:
        q1, q2, q3, q4, q5, q6 = joint_values
        q1 = -1 * q1
        q2 = q2 - math.pi / 2
        q3 = q3 + math.pi / 2
        q4 = q4 * -1
        q6 = q6 * -1 + math.pi / 2
        return [q1, q2, q3, q4, q5, q6]

    def _post_process(self, solutions: list[list[float]]) -> list[list[float]]:
        for q1, q2, q3, q4, q5, q6 in solutions:
            q1 = -1 * q1
            q2 = q2 + math.pi / 2
            q3 = q3 - math.pi / 2
            q4 = q4 * -1
            q6 = q6 * -1 + math.pi / 2
            yield [q1, q2, q3, q4, q5, q6]
