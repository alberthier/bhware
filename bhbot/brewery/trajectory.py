#!/usr/bin/env python
# encoding: utf-8

class Pose(object):

    def __init__(self, x = 0.0, y = 0.0, angle = None):
        self.x = x
        self.y = y
        self.angle = angle


    def __repr__(self):
        return "[{0}, {1}, {2}]".format(self.x, self.y, self.angle)
