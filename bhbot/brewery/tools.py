#!/usr/bin/env python
# encoding: utf-8

import math


EPSILON = 1e-6


def quasi_equal(f1, f2):
    return abs(f1 - f2) < EPSILON


def quasi_null(f1):
    return quasi_equal(f1, 0.0)
