import math


def get_smaller_angle(angle):  # delete
    angle1 = angle % (2 * math.pi)
    angle2 = angle1 - 2 * math.pi
    return angle1 if angle1 < math.fabs(angle2) else angle2


def smallest_joint_angles(joint_angles):  # move to configuration ?
    return [get_smaller_angle(j) for j in joint_angles]


def fit_within_bounds(angle, lower, upper):
    while angle < lower:
        angle += 2 * math.pi
    while angle > upper:
        angle -= 2 * math.pi
    assert(angle >= lower and angle <= upper)
    return angle
