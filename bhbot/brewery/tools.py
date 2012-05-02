#!/usr/bin/env python
# encoding: utf-8

import math


EPSILON = 1e-6


def quasi_equal(f1, f2):
    return abs(f1 - f2) < EPSILON


def quasi_null(f1):
    return quasi_equal(f1, 0.0)


def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def manathan_distance(x1, y1, x2, y2):
    return abs(x2 - x1) + abs(y2 - y1)


def angle_between(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1)


def normalize_angle(a):
    if a < 0:
        return a % (-2.0 * math.pi)
    else:
        return a % (2.0 * math.pi)


def segment_affine_params(x1, y1, x2, y2):
    """
        Returns (a, b) parameters of the y = a * x + b corresponding line
    """
    if quasi_equal(x1, x2):
       return (None, None)
    else:
        a = (y2 - y1) / (x2 - x1)
        b = y1 - a * x1
        return (a, b)


def segment_contains_full(x1, y1, x2, y2, a, b, px, py):
    nx1 = min(x1, x2)
    ny1 = min(y1, y2)
    nx2 = max(x1, x2)
    ny2 = max(y1, y2)

    if a == None:
        ok = quasi_equal(px, nx1) and (py > ny1 and py < ny2)
    else:
        ok = quasi_equal(a * px + b, py)
        if ok:
            if quasi_equal(ny1, ny2):
                ok = quasi_equal(ny1, py)
            else:
                ok = py > ny1 and py < ny2
            ok = ok and px > nx1 and px < nx2
    return ok


def segment_contains(x1, y1, x2, y2, px, py):
    (a, b) = segment_affine_params(x1, y1, x2, y2)
    segment_contains_full(x1, y1, x2, y2, a, b, px, py)


def segment_intersects_segment(px1, py1, px2, py2, qx1, qy1, qx2, qy2):
    (pa, pb) = segment_affine_params(px1, py1, px2, py2)
    (qa, qb) = segment_affine_params(qx1, qy1, qx2, qy2)

    if pa is None and qa is None:
        return False
    if pa is None:
        cross_x = px1
        a = qa
        b = qb
    else:
        a = pa
        b = pb
    if qa is None:
        cross_x = qx1
    if not pa is None and not qa is None:
        if quasi_equal(pa, qa):
            return False
        else:
            cross_x = (qb - pb) / (pa - qa)

    cross_y = a * cross_x + b

    ok = segment_contains_full(px1, py1, px2, py2, pa, pb, cross_x, cross_y)
    ok = ok and segment_contains_full(qx1, qy1, qx2, qy2, qa, qb, cross_x, cross_y)
    return ok
