#!/usr/bin/env python
# encoding: utf-8

import math

import trajectory
import packets
import logger

import tools

from definitions import *




class Robot(object):

    def __init__(self, event_loop):
        self.pose = trajectory.Pose(0.0, 0.0, 0.0)
        self.team = TEAM_UNKNOWN
        self.event_loop = event_loop
        self.stored_piece_count = 0


    def move(self, dx, dy, direction = DIRECTION_FORWARD, reference_team = TEAM_UNKNOWN):
        y = self.convert_y(self.pose.y, reference_team)
        y = self.convert_y(y + dy, reference_team)
        return self.goto(self.pose.x + dx, y, None, direction, self.team)


    def move_to(self, x, y, direction = DIRECTION_FORWARD, reference_team = TEAM_UNKNOWN):
        y = self.convert_y(y, reference_team)
        return self.goto(x, y, None, direction, self.team)


    def forward(self, distance, reference_team = TEAM_UNKNOWN):
        dx = math.cos(self.pose.angle) * distance
        dy = math.sin(self.pose.angle) * distance
        return self.goto(self.pose.x + dx, self.pose.y + dy, None, DIRECTION_FORWARD, self.team)


    def backward(self, distance, reference_team = TEAM_UNKNOWN):
        dx = -math.cos(self.pose.angle) * distance
        dy = -math.sin(self.pose.angle) * distance
        return self.goto(self.pose.x + dx, self.pose.y + dy, None, DIRECTION_BACKWARD, self.team)


    def look_at(self, x, y, reference_team = TEAM_UNKNOWN):
        current_y = self.convert_y(self.pose.y, reference_team)
        dx = x - self.pose.x
        dy = y - current_y
        return self.rotate_to(math.atan2(dy, dx), reference_team)


    def look_at_opposite(self, x, y, reference_team = TEAM_UNKNOWN):
        current_y = self.convert_y(self.pose.y, reference_team)
        dx = x - self.pose.x
        dy = y - current_y
        return self.rotate_to(math.atan2(dy, dx) + math.pi, reference_team)

    def stop(self):
        packet = packets.Stop()
        self.event_loop.send_packet(packet)

    def rotate(self, da, reference_team = TEAM_UNKNOWN):
        angle = self.convert_angle(self.pose.angle, reference_team)
        angle += da
        angle = self.convert_angle(angle, reference_team)
        return self.goto(None, None, angle, DIRECTION_FORWARD, self.team)


    def rotate_to(self, angle, reference_team = TEAM_UNKNOWN):
        angle = self.convert_angle(angle, reference_team)
        return self.goto(None, None, angle, DIRECTION_FORWARD, self.team)


    def goto(self, x, y, angle, direction, reference_team = TEAM_UNKNOWN):
        y = self.convert_y(y, reference_team)
        angle = self.convert_angle(angle, reference_team)
        packet = packets.Goto()
        packet.movement = None
        if x == None or tools.quasi_equal(x, self.pose.x):
            x = self.pose.x
        else:
            packet.movement = MOVEMENT_LINE
        if y == None or tools.quasi_equal(y, self.pose.y):
            y = self.pose.y
        else:
            packet.movement = MOVEMENT_LINE
        if angle == None or tools.quasi_equal(angle, self.pose.angle):
            angle = self.pose.angle
            if packet.movement == None:
                self.event_loop.inject_goto_finished()
                return None
        elif packet.movement == MOVEMENT_LINE:
            packet.movement = MOVEMENT_MOVE
        else:
            packet.movement = MOVEMENT_ROTATE

        packet.direction = direction
        packet.points = [trajectory.Pose(x, y, angle)]
        self.event_loop.send_packet(packet)
        return packet


    def follow(self, points, direction, reference_team = TEAM_UNKNOWN):
        packet = packets.Goto()

        for p in points:
            converted_point = trajectory.Pose(p.x, self.convert_y(p.y), self.convert_angle(p.angle))
            packet.points.append(converted_point)

        packet.movement = MOVEMENT_MOVE
        packet.direction = direction
        self.event_loop.send_packet(packet)
        return packet


    def convert_y(self, y, reference_team):
        if y == None or reference_team == TEAM_UNKNOWN or self.team == TEAM_UNKNOWN or reference_team == self.team:
            return y
        else:
            return FIELD_WIDTH - y


    def convert_angle(self, angle, reference_team):
        if angle == None or reference_team == TEAM_UNKNOWN or self.team == TEAM_UNKNOWN or reference_team == self.team:
            return angle
        else:
            return math.atan2(-math.sin(angle), math.cos(angle))


    def convert_sensor(self, sensor, reference_team):
        if reference_team == TEAM_UNKNOWN or self.team == TEAM_UNKNOWN or reference_team == self.team:
            return sensor
        else:
            if sensor == SENSOR_LEFT_BOTTOM:
                return SENSOR_RIGHT_BOTTOM
            elif sensor == SENSOR_LEFT_TOP:
                return SENSOR_RIGHT_TOP
            elif sensor == SENSOR_RIGHT_BOTTOM:
                return SENSOR_LEFT_BOTTOM
            elif sensor == SENSOR_RIGHT_TOP:
                return SENSOR_LEFT_TOP

