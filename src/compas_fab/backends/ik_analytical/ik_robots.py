import math
from compas.geometry import Point
from compas.geometry import Frame
from compas_fab.robots import Configuration
from compas_fab.backends.ik_analytical import fit_within_bounds, get_smaller_angle
from compas_fab.backends.ik_analytical import inverse_kinematics_spherical_wrist
from compas_fab.backends.ik_analytical import forward_kinematics_spherical_wrist


class IK(object):

    def __init__(self, joints):
        pass

def reduce_to_configurations_between_bounds(configurations, joints):
    j1, j2, j3, j4, j5, j6 = joints
    configurations_within_bounds = []
    for c in configurations:
        a1, a2, a3, a4, a5, a6 = c.values
        try:
            a1 = fit_within_bounds(a1, j1.limit.lower, j1.limit.upper)
            a2 = fit_within_bounds(a2, j2.limit.lower, j2.limit.upper)
            a3 = fit_within_bounds(a3, j3.limit.lower, j3.limit.upper)
            a4 = fit_within_bounds(a4, j4.limit.lower, j4.limit.upper)
            a5 = fit_within_bounds(a5, j5.limit.lower, j5.limit.upper)
            a6 = fit_within_bounds(a6, j6.limit.lower, j6.limit.upper)
            configurations_within_bounds.append(Configuration.from_revolute_values([a1, a2, a3, a4, a5, a6]))
        except AssertionError:
            pass
    return configurations_within_bounds

def calculate_small_angles(A1, A2, A3, A4, A5, A6):
    for i in range(8):
        A1[i] = get_smaller_angle(A1[i])
        A2[i] = get_smaller_angle(A2[i])
        A3[i] = get_smaller_angle(A3[i])
        A4[i] = get_smaller_angle(A4[i])
        A5[i] = get_smaller_angle(A5[i])
        A6[i] = get_smaller_angle(A5[i])
    return A1, A2, A3, A4, A5, A6

def joint_angles_to_configurations(A1, A2, A3, A4, A5, A6):
    return [Configuration.from_revolute_values([a1, a2, a3, a4, a5, a6]) for a1, a2, a3, a4, a5, a6 in zip(A1, A2, A3, A4, A5, A6)]

def ik_staubli_txl60(frame_rcf, joints=None):

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

    configurations = joint_angles_to_configurations(A1, A2, A3, A4, A5, A6)

    return configurations

def ik_abb_irb4600_40_255(frame_rcf, joints=None):

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

    configurations = joint_angles_to_configurations(A1, A2, A3, A4, A5, A6)

    if joints:
        return reduce_to_configurations_between_bounds(configurations, joints)
    else:
        return configurations


if __name__ == "__main__":

    frame_rcf = Frame((0.600, 0.700, 1.000), (-1., 0., 0.), (0., 1., 0.))
    joints = []
    configurations = ik_abb_irb4600_40_255(frame_rcf, joints)

    print("")
    frame_rcf = Frame((0.200, 0.200, 0.200), (-1., 0., 0.), (0., 1., 0.))
    configurations = ik_staubli_txl60(frame_rcf, joints=None)
    