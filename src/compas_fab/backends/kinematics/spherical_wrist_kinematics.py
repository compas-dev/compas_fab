
import math
from compas.geometry import Point
from .spherical_wrist import forward_kinematics_spherical_wrist
from .spherical_wrist import inverse_kinematics_spherical_wrist


class SphericalWristKinematics(object):
    """
    """

    def __init__(self, points):
        self.points = points

    def forward(self, joint_values):
        return forward_kinematics_spherical_wrist(joint_values, self.points)

    def inverse(self, frame_rcf):
        angles = inverse_kinematics_spherical_wrist(frame_rcf, self.points)
        return self._post_process(angles)

    def _post_process(self, angles):
        return angles


class Staubli_TX2_60L_Kinematics(SphericalWristKinematics):
    """
    """

    def __init__(self):
        points = [Point(0.000, 0.000, 0.375),
                  Point(0.000, 0.020, 0.775),
                  Point(0.450, 0.020, 0.775),
                  Point(0.520, 0.020, 0.775)]
        super(Staubli_TX2_60L_Kinematics, self).__init__(points)

    def _post_process(self, angles):
        A1, A2, A3, A4, A5, A6 = angles
        for i in range(8):
            A1[i] = -1 * A1[i]
            A2[i] = A2[i] + math.pi / 2
            A4[i] = A4[i] * -1
            A6[i] = A6[i] * -1 + math.pi / 2
        return (A1, A2, A3, A4, A5, A6)


class ABB_IRB_4600_40_255_Kinematics(SphericalWristKinematics):
    """
    """

    def __init__(self):
        points = [Point(0.175, 0.000, 0.495),
                  Point(0.175, 0.000, 1.590),
                  Point(1.446, 0.000, 1.765),
                  Point(1.581, 0.000, 1.765)]
        super(ABB_IRB_4600_40_255_Kinematics, self).__init__(points)

    def _post_process(self, angles):
        A1, A2, A3, A4, A5, A6 = angles
        for i in range(8):
            A1[i] = -1 * A1[i]
            A2[i] = A2[i] + math.pi / 2
            A3[i] = A3[i] - math.pi / 2
            A4[i] = -1 * A4[i]
            A6[i] = -1 * A6[i] + math.pi/2
        return (A1, A2, A3, A4, A5, A6)
