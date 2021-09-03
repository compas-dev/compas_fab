import math


def get_smaller_angle(angle):
    angle1 = angle % (2 * math.pi)
    angle2 = angle1 - 2 * math.pi
    return angle1 if angle1 < math.fabs(angle2) else angle2


def fit_within_bounds(angle, lower, upper):
    angle = get_smaller_angle(angle)
    if angle < lower:
        angle += 2 * math.pi
    if angle > upper:
        angle -= 2 * math.pi
    assert(angle >= lower and angle <= upper)
    return angle
