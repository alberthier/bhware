#!/usr/bin/env python
# encoding: utf-8

from definitions import *




class Pose(object):

    def __init__(self, x = 0.0, y = 0.0, angle = None):
        self.x = x
        self.y = y
        self.angle = angle


    def __repr__(self):
        return "[{0}, {1}, {2}]".format(self.x, self.y, self.angle)


    def __eq__(self, other):
        if other == None:
            return False
        return self.x == other.x and self.y == other.y and self.angle == other.angle




class Cell(object):

    def __init__(self, x, y) :
        self.x = x
        self.y = y


    def center_left(self) :
        x = FIELD_GREEN_ZONE_WIDTH + FIELD_CELL_SIZE * self.x
        y = FIELD_CELL_SIZE * (self.y + 0.5)
        return y,x


    def center_right(self) :
        x = FIELD_GREEN_ZONE_WIDTH + FIELD_CELL_SIZE * (self.x + 1)
        y = FIELD_CELL_SIZE * (self.y + 0.5)
        return y,x


    def down_right(self) :
        x = FIELD_GREEN_ZONE_WIDTH + FIELD_CELL_SIZE * (self.x + 1)
        y = FIELD_CELL_SIZE * (self.y + 1)
        return y,x


    def center_middle(self) :
        x = FIELD_GREEN_ZONE_WIDTH + FIELD_CELL_SIZE * (self.x + 0.5)
        y = FIELD_CELL_SIZE * (self.y + 0.5)
        return y,x


    def down_middle(self) :
        x = FIELD_GREEN_ZONE_WIDTH + FIELD_CELL_SIZE * (self.x + 0.5)
        y = FIELD_CELL_SIZE * (self.y + 1)
        return y,x
