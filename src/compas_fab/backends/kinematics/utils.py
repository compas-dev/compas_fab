import math


def get_smaller_angle(angle):
    angle = angle % (2 * math.pi)
    sign = 1 if angle >= 0 else -1
    if math.fabs(angle) > math.pi:
        angle = angle - sign * 2*math.pi
    return angle


def fit_within_bounds(angle, lower, upper):
    angle = get_smaller_angle(angle)
    if angle < lower:
        angle += 2 * math.pi
    if angle > upper:
        angle -= 2 * math.pi
    assert(angle >= lower and angle <= upper)
    return angle
