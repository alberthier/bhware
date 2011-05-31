#!/usr/bin/env python
# encoding: utf-8


import commonstates
from definitions import *




class FigureDetector(object):

    PIECE_NONE = 0

    def __init__(self, robot):

        self.x_coords = [[0.690,                 0.970,                 1.250,                 1.530,                 1.810                ],
                         [FIELD_CELL_SIZE * 1.0, FIELD_CELL_SIZE * 2.0, FIELD_CELL_SIZE * 3.0, FIELD_CELL_SIZE * 4.0, FIELD_CELL_SIZE * 5.0],
                         [FIELD_CELL_SIZE * 1.0, FIELD_CELL_SIZE * 2.0, FIELD_CELL_SIZE * 3.0, FIELD_CELL_SIZE * 4.0, FIELD_CELL_SIZE * 5.0]]
        self.elements = [[False, False, False, False, False],
                         [False, False, False, False, False],
                         [False, False, False, False, False]]
                                # We don't want to ever take the last piece of the first line.
        self.taken_elements = [[False, False, False, False, True],
                               [False, False, False, False, False],
                               [False, False, False, False, False]]
        self.reference_sensor = None
        self.robot = robot
        self.column = None


    def enable(self, reference_sensor, column):
        self.reference_sensor = reference_sensor
        self.column_index = column


    def disable(self, fix_column):
        self.reference_sensor = None
        figure_count = 0
        elements_column = self.elements[self.column_index]
        for flag in elements_column:
            if flag:
                figure_count += 1
        if fix_column and figure_count < 2:
            elements_column[4] = True


    def on_piece_detected(self, start_pose, start_distance, end_pose, end_distance, sensor, angle):
        if sensor == self.reference_sensor:
            # piece detected bug workaround
            # end_pose = self.robot.pose
            center_x = (start_pose.x + end_pose.x) / 2.0
            center_angle = (start_pose.angle + end_pose.angle) / 2.0
            if math.cos(center_angle) > 0:
                center_x += ROBOT_CENTER_TO_LATERAL_SENSOR_DISTANCE
            else:
                center_x -= ROBOT_CENTER_TO_LATERAL_SENSOR_DISTANCE
            self.detected_at(self.column_index, center_x)


    def detected_at(self, column_index, x):
        min_diff = 9999.0
        min_idx = -1
        idx = 0
        x_coords_column = self.x_coords[column_index]
        elements_column = self.elements[column_index]

        for x_coord in x_coords_column:
            diff = abs(x_coord - x)
            if diff < min_diff:
                min_diff = diff
                min_idx = idx
            idx += 1
        if (min_idx != -1):
            elements_column[min_idx] = True


    def get_elements_count(self, column_index):
        count = 0
        for elt in self.elements[column_index]:
            if elt:
                count += 1
        return count


    def pop_nearest_match_x(self, column_index):
        min_diff = 9999.0
        min_idx = -1
        x_coords_column = self.x_coords[column_index]
        elements_column = self.elements[column_index]
        taken_elements_column = self.taken_elements[column_index]

        for idx in xrange(len(elements_column)):
            if not taken_elements_column[idx] and elements_column[idx]:
                x_coord = x_coords_column[idx]
                diff = abs(x_coord - self.robot.pose.x)
                if diff < min_diff:
                    min_diff = diff
                    min_idx = idx

        if min_idx != -1:
            taken_elements_column[min_idx] = True
        else:
            # arg nothing detected
            return None

        return x_coords_column[min_idx]


    def pop_nearest_green_zone_pawn_x(self):
        min_diff = 9999.0
        min_idx = -1

        x_coords_column = self.x_coords[0]
        elements_column = self.elements[0]
        taken_elements_column = self.taken_elements[0]

        for idx in xrange(len(elements_column)):
            # No figure detected and not already taken
            if not elements_column[idx] and not taken_elements_column[idx]:
                x_coord = x_coords_column[idx]
                diff = abs(x_coord - self.robot.pose.x)
                if diff < min_diff:
                    min_diff = diff
                    min_idx = idx

        if min_idx != -1:
            taken_elements_column[min_idx] = True
        else:
            # arg nothing detected
            return None

        return x_coords_column[min_idx]
