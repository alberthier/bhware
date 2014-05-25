# encoding: utf-8

import math


EPSILON = 1e-6


def quasi_equal(f1, f2):
    return abs(f1 - f2) < EPSILON


def quasi_null(f1):
    return quasi_equal(f1, 0.0)


def is_between(a, b, x):
    if a < b:
        ok = a < x and x < b
    else:
        ok = b < x and x < a
    return ok or quasi_equal(a, x) or quasi_equal(b, x)


def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def manathan_distance(x1, y1, x2, y2):
    return abs(x2 - x1) + abs(y2 - y1)


def angle_between(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1)


def normalize_angle(a):
    na = a % (2.0 * math.pi)
    if na > math.pi:
        na -= 2.0 * math.pi
    return na

def sym_x(coord_x):
    return 2.0 - coord_x

def sym_y(coord_y):
    return 3.0 - coord_y


def sym_angle(angle):
    return -angle

def angle_minus_pi_to_pi(a):
    if a > math.pi :
        return a - 2 * math.pi
    if a <= -math.pi :
        return a + 2 * math.pi
    return a

def compare_angles(a1, a2, epsilon):
    return abs(angle_minus_pi_to_pi((angle_minus_pi_to_pi(a1) - angle_minus_pi_to_pi(a2)))) < epsilon
