
import math
from compas.geometry import Point
from compas.geometry import Frame
from compas_fab.robots import Configuration
from compas_fab.backends.ik_solver import inverse_kinematics_spherical_wrist


def joint_angles_to_configurations(A1, A2, A3, A4, A5, A6):
    return [Configuration.from_revolute_values([a1, a2, a3, a4, a5, a6]) for a1, a2, a3, a4, a5, a6 in zip(A1, A2, A3, A4, A5, A6)]


def ik_staubli_txl60(frame_rcf):

    p1 = Point(0.000, 0.000, 0.375)
    p2 = Point(0.000, 0.020, 0.775)
    p3 = Point(0.450, 0.020, 0.775)
    p4 = Point(0.520, 0.020, 0.775)

    A1, A2, A3, A4, A5, A6 = inverse_kinematics_spherical_wrist(p1, p2, p3, p4, frame_rcf)

    for i in range(8):
        A1[i] = -1 * A1[i]
        A2[i] = A2[i] + math.pi / 2
        A4[i] = A4[i] * -1
        A6[i] = A6[i] * -1 + math.pi / 2

    return joint_angles_to_configurations(A1, A2, A3, A4, A5, A6)


def ik_abb_irb4600_40_255(frame_rcf):

    p1 = Point(0.175, 0.000, 0.495)
    p2 = Point(0.175, 0.000, 1.590)
    p3 = Point(1.446, 0.000, 1.765)
    p4 = Point(1.581, 0.000, 1.765)

    A1, A2, A3, A4, A5, A6 = inverse_kinematics_spherical_wrist(p1, p2, p3, p4, frame_rcf)

    for i in range(8):
        A1[i] = -1 * A1[i]
        A2[i] = A2[i] + math.pi / 2
        A3[i] = A3[i] - math.pi / 2
        A4[i] = -1 * A4[i]
        A6[i] = -1 * A6[i] + math.pi/2

    return joint_angles_to_configurations(A1, A2, A3, A4, A5, A6)


if __name__ == "__main__":

    frame_rcf = Frame((0.600, 0.700, 1.000), (-1., 0., 0.), (0., 1., 0.))
    configurations = ik_abb_irb4600_40_255(frame_rcf)

    print("")
    frame_rcf = Frame((0.200, 0.200, 0.200), (-1., 0., 0.), (0., 1., 0.))
    configurations = ik_staubli_txl60(frame_rcf)
