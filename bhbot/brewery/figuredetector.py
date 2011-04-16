#!/usr/bin/env python
# encoding: utf-8


from definitions import *


class FigureDetector(object):

    def __init__(self):
        self.enabled = False
        self.elements = [(690.0, False), (970.0, False), (1250.0, False), (1530.0, False), (1810.0, False)]


    def enable(self):
        self.enabled = True


    def disable(self):
        self.enabled = False


    def on_piece_detected(start_pose, start_distance, end_pose, end_distance, sensor, angle):
        if (self.enabled):
            if (sensor == SENSOR_LEFT_TOP || sensor == SENSOR_RIGHT_TOP):
                center_x = (start_pose.x + end_pose.x) / 2.0
                min = 9999.0
                min_idx = -1
                idx = 0
                for elt in elements:
                    diff = abs(elt[0] - center_x)
                    if diff < min:
                        min = diff
                        min_idx = idx
                    idx += 1
                if (min_idx != -1):
                    self.elements[min_idx][1] = True
