
import math
from compas.geometry import Point
from compas_fab.backends.kinematics.spherical_wrist import forward_kinematics_spherical_wrist
from compas_fab.backends.kinematics.spherical_wrist import inverse_kinematics_spherical_wrist


class SphericalWristKinematics(object):
    """
    """

    def __init__(self, points):
        self.points = points

    def forward(self, joint_values):
        joint_values = self._pre_process(joint_values)
        return forward_kinematics_spherical_wrist(joint_values, self.points)

    def inverse(self, frame_rcf):
        solutions = inverse_kinematics_spherical_wrist(frame_rcf, self.points)
        return list(self._post_process(solutions))
        # return solutions

    def _post_process(self, solutions):
        return solutions

    def _pre_process(self, joint_values):
        return joint_values


class Staubli_TX2_60L(SphericalWristKinematics):
    """
    """

    def __init__(self):
        points = [Point(0.000, 0.000, 0.375),
                  Point(0.000, 0.020, 0.775),
                  Point(0.450, 0.020, 0.775),
                  Point(0.520, 0.020, 0.775)]
        super(Staubli_TX2_60L, self).__init__(points)

    def _pre_process(self, joint_values):
        q1, q2, q3, q4, q5, q6 = joint_values
        q1 = -1 * q1
        q2 = q2 - math.pi / 2
        q4 = q4 * -1
        q6 = q6 * -1 + math.pi / 2
        return [q1, q2, q3, q4, q5, q6]

    def _post_process(self, solutions):
        for q1, q2, q3, q4, q5, q6 in solutions:
            q1 = -1 * q1
            q2 = q2 + math.pi / 2
            q4 = q4 * -1
            q6 = q6 * -1 + math.pi / 2
            yield [q1, q2, q3, q4, q5, q6]


class ABB_IRB_4600_40_255(SphericalWristKinematics):
    """
    """

    def __init__(self):
        points = [Point(0.175, 0.000, 0.495),
                  Point(0.175, 0.000, 1.590),
                  Point(1.446, 0.000, 1.765),
                  Point(1.581, 0.000, 1.765)]
        super(ABB_IRB_4600_40_255, self).__init__(points)

    def _pre_process(self, joint_values):
        q1, q2, q3, q4, q5, q6 = joint_values
        q1 = -1 * q1
        q2 = q2 - math.pi / 2
        q3 = q3 + math.pi / 2
        q4 = q4 * -1
        q6 = q6 * -1 + math.pi / 2
        return [q1, q2, q3, q4, q5, q6]

    def _post_process(self, solutions):
        for q1, q2, q3, q4, q5, q6 in solutions:
            q1 = -1 * q1
            q2 = q2 + math.pi / 2
            q3 = q3 - math.pi / 2
            q4 = q4 * -1
            q6 = q6 * -1 + math.pi / 2
            yield [q1, q2, q3, q4, q5, q6]


if __name__ == "__main__":
    from compas.geometry import allclose

    kin = ABB_IRB_4600_40_255()
    q = [0.2, 0.5, 1.4, 1.3, 2.6, 2.3]
    frame = kin.forward(q)
    sol = kin.inverse(frame)
    assert(allclose(sol[0], q))

    kin = Staubli_TX2_60L()
    q = [0.2, 0.5, 1.4, 1.3, 2.6, 2.3]
    frame = kin.forward(q)
    sol = kin.inverse(frame)
    assert(allclose(sol[0], q))
