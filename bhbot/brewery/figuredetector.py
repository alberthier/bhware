#!/usr/bin/env python
# encoding: utf-8


import commonstates
import packets
from definitions import *




class FigureDetector(object):

    def __init__(self, robot):

        self.x_coords = [[0.690, FIELD_CELL_SIZE * 1.0, FIELD_CELL_SIZE * 1.0],
                         [0.970, FIELD_CELL_SIZE * 2.0, FIELD_CELL_SIZE * 2.0],
                         [1.250, FIELD_CELL_SIZE * 3.0, FIELD_CELL_SIZE * 3.0],
                         [1.530, FIELD_CELL_SIZE * 4.0, FIELD_CELL_SIZE * 4.0],
                         [1.810, FIELD_CELL_SIZE * 5.0, FIELD_CELL_SIZE * 5.0]]
        self.elements = [[False, False, False],
                         [False, False, False],
                         [False, False, False],
                         [False, False, False],
                         [False, False, False]]
        self.reference_sensor = None
        self.robot = robot
        self.column = None


    def enable(self, reference_sensor, column):
        self.reference_sensor = reference_sensor
        self.column = column
        return commonstates.EnableLateralSensors()


    def disable(self):
        self.reference_sensor = None
        self.enabled_pose = None
        figure_count = 0
        if self.column == 0:
            for elt in self.elements:
                if elt[self.column]:
                    figure_count += 1
            if figure_count < 2:
                self.elements[4][self.column] = True
        return commonstates.DisableLateralSensors()


    def on_piece_detected(self, start_pose, start_distance, end_pose, end_distance, sensor, angle):
        if sensor == self.reference_sensor:
            center_x = (start_pose.x + end_pose.x) / 2.0
            center_angle = (start_pose.angle + end_pose.angle) / 2.0
            if math.cos(center_angle) > 0:
                center_x += ROBOT_CENTER_TO_LATERAL_SENSOR_DISTANCE
            else:
                center_x -= ROBOT_CENTER_TO_LATERAL_SENSOR_DISTANCE
            self.detected_at(self.column, center_x)


    def detected_at(self, column, x):
        min = 9999.0
        min_idx = -1
        idx = 0
        for x_coord in self.x_coords:
            diff = abs(x_coord[column] - x)
            if diff < min:
                min = diff
                min_idx = idx
            idx += 1
        if (min_idx != -1):
            self.elements[min_idx][column] = True


    def get_first_match_x(self, column):
        idx = 0
        for elt in self.elements:
            if elt[column]:
                return self.x_coords[idx][column]
            idx += 1
        return None


    def get_second_match_x(self, column):
        idx = 0
        for elt in reversed(self.elements):
            if elt[column]:
                return self.x_coords[idx][column]
            idx += 1
        return None


    def get_green_zone_pawns_x(self):
        pawns_x = []
        idx = 0
        for elt in self.elements:
            if not elt[0]:
                pawns_x.append(self.x_coords[idx][0])
            idx += 1
        return pawns_x
