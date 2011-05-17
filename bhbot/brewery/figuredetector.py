#!/usr/bin/env python
# encoding: utf-8


from definitions import *


class FigureDetector(object):

    def __init__(self):
        self.elements = [[0.690, False], [0.970, False], [1.250, False], [1.530, False], [1.810, False]]
        self.reference_sensor = None


    def enable(self, reference_sensor):
        self.reference_sensor = reference_sensor
        for elt in self.elements:
            elt[1] = False


    def disable(self):
        self.reference_sensor = None
        figure_count = 0
        for elt in self.elements:
            if elt[1]:
                figure_count += 1
        if figure_count < 2:
            self.elements[4][1] = True


    def on_piece_detected(self, start_pose, start_distance, end_pose, end_distance, sensor, angle):
        if sensor == self.reference_sensor:
            center_x = (start_pose.x + end_pose.x) / 2.0
            min = 9999.0
            min_idx = -1
            idx = 0
            for elt in self.elements:
                diff = abs(elt[0] - center_x)
                if diff < min:
                    min = diff
                    min_idx = idx
                idx += 1
            if (min_idx != -1):
                self.elements[min_idx][1] = True


    def get_first_figure_x(self):
        for elt in self.elements:
            if elt[1]:
                return elt[0]
        return None


    def get_second_figure_x(self):
        for elt in reversed(self.elements):
            if elt[1]:
                return elt[0]
        return None
