#!/usr/bin/env python
# encoding: utf-8

import math

from definitions import *




class Pose(object):

    def __init__(self, x = 0.0, y = 0.0, angle = None):
        self.x = x
        self.y = y
        self.angle = angle


    def look_at(self, pose):
        return math.atan2(pose.y - self.y, pose.x - self.x)


    def __repr__(self):
        return "Pose({0}, {1}, {2})".format(self.x, self.y, self.angle)


    def __eq__(self, other):
        if other == None:
            return False
        return self.x == other.x and self.y == other.y and self.angle == other.angle




class Cell(object):

    def __init__(self, x, y) :
        self.x = x
        self.y = y


    def top_left(self):
        y = FIELD_GREEN_ZONE_WIDTH + FIELD_CELL_SIZE * self.y
        x = FIELD_CELL_SIZE * self.x
        return x, y


    def top_middle(self):
        y = FIELD_GREEN_ZONE_WIDTH + FIELD_CELL_SIZE * (self.y + 0.5)
        x = FIELD_CELL_SIZE * self.x
        return x, y


    def top_right(self):
        y = FIELD_GREEN_ZONE_WIDTH + FIELD_CELL_SIZE * (self.y + 1)
        x = FIELD_CELL_SIZE * self.x
        return x, y


    def center_left(self) :
        y = FIELD_GREEN_ZONE_WIDTH + FIELD_CELL_SIZE * self.y
        x = FIELD_CELL_SIZE * (self.x + 0.5)
        return x, y


    def center_middle(self) :
        y = FIELD_GREEN_ZONE_WIDTH + FIELD_CELL_SIZE * (self.y + 0.5)
        x = FIELD_CELL_SIZE * (self.x + 0.5)
        return x, y


    def center_right(self) :
        y = FIELD_GREEN_ZONE_WIDTH + FIELD_CELL_SIZE * (self.y + 1)
        x = FIELD_CELL_SIZE * (self.x + 0.5)
        return x, y


    def bottom_left(self):
        y = FIELD_GREEN_ZONE_WIDTH + FIELD_CELL_SIZE * self.y
        x = FIELD_CELL_SIZE * (self.x + 1)
        return x, y


    def bottom_middle(self) :
        y = FIELD_GREEN_ZONE_WIDTH + FIELD_CELL_SIZE * (self.y + 0.5)
        x = FIELD_CELL_SIZE * (self.x + 1)
        return x, y


    def bottom_right(self) :
        y = FIELD_GREEN_ZONE_WIDTH + FIELD_CELL_SIZE * (self.y + 1)
        x = FIELD_CELL_SIZE * (self.x + 1)
        return x, y


