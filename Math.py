import math


def bound_angle_plus_minus_pi(angle):
    while angle >= math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def bound_angle_0_to_2pi_radians(angle):
    while angle >= 2.0 * math.pi:
        angle -= 2.0 * math.pi
    while angle < 0.0:
        angle += 2.0 * math.pi
    return angle


def get_difference_in_angle_radians(start, end):
    return bound_angle_plus_minus_pi(end - start)


def deg_to_rad(deg):
    return deg * math.pi / 180.0


def rad_to_deg(rad):
    return rad * 180.0 / math.pi
