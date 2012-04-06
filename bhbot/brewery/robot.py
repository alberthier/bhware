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
        self.tank_full = False


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
        return packet


    def rotate(self, da, reference_team = TEAM_UNKNOWN):
        angle = self.convert_angle(self.pose.angle, reference_team)
        angle += da
        angle = self.convert_angle(angle, reference_team)
        return self.goto(None, None, angle, DIRECTION_FORWARD, self.team)


    def rotate_to(self, angle, reference_team = TEAM_UNKNOWN):
        angle = self.convert_angle(angle, reference_team)
        return self.goto(None, None, angle, DIRECTION_FORWARD, self.team)


    def goto_looking_at(self, x, y, look_at_x, look_at_y, direction = DIRECTION_FORWARD, reference_team = TEAM_UNKNOWN):
        dest_y = self.convert_y(y, reference_team)
        dest_look_at_y = self.convert_y(look_at_y, reference_team)
        dx = look_at_x - x
        dy = dest_look_at_y - dest_y
        return self.goto(x, dest_y, math.atan2(dy, dx), direction, self.team)


    def goto(self, x, y, angle, direction = DIRECTION_FORWARD, reference_team = TEAM_UNKNOWN):
        y = self.convert_y(y, reference_team)
        angle = self.convert_angle(angle, reference_team)
        if angle is not None:
            angle = math.fmod(angle, 2.0 * math.pi)
        packet = packets.Goto()
        packet.movement = None
        if x is None or tools.quasi_equal(x, self.pose.x):
            x = self.pose.x
        else:
            packet.movement = MOVEMENT_LINE
        if y is None or tools.quasi_equal(y, self.pose.y):
            y = self.pose.y
        else:
            packet.movement = MOVEMENT_LINE
        if angle is None or tools.quasi_equal(angle, self.pose.angle):
            angle = self.pose.angle
            if packet.movement is None:
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


    def follow(self, points, direction = DIRECTION_FORWARD, reference_team = TEAM_UNKNOWN):
        packet = packets.Goto()

        for p in points:
            converted_point = trajectory.Pose(p.x, self.convert_y(p.y, reference_team), self.convert_angle(p.angle, reference_team))
            packet.points.append(converted_point)

        packet.movement = MOVEMENT_MOVE
        packet.direction = direction
        self.event_loop.send_packet(packet)
        return packet


    def is_looking_at(self, x, y, reference_team = TEAM_UNKNOWN):
        current_y = self.convert_y(self.pose.y, reference_team)
        dx = x - self.pose.x
        dy = y - current_y
        angle = self.convert_angle(math.atan2(dy, dx), reference_team) % (2.0 * math.pi)
        current_angle = self.pose.angle % (2.0 * math.pi)
        return abs(current_angle - angle) < (math.pi / 16.0)


    def is_looking_at_opposite(self, x, y, reference_team = TEAM_UNKNOWN):
        current_y = self.convert_y(self.pose.y, reference_team)
        dx = x - self.pose.x
        dy = y - current_y
        angle = self.convert_angle(math.atan2(dy, dx), reference_team) % (2.0 * math.pi)
        current_opposite_angle = (self.pose.angle + math.pi) % (2.0 * math.pi)
        return abs(current_opposite_angle - angle) < (math.pi / 16.0)


    def convert_y(self, y, reference_team):
        if y is None or reference_team == TEAM_UNKNOWN or self.team == TEAM_UNKNOWN or reference_team == self.team:
            return y
        else:
            return FIELD_Y_SIZE - y


    def convert_angle(self, angle, reference_team):
        if angle is None or reference_team == TEAM_UNKNOWN or self.team == TEAM_UNKNOWN or reference_team == self.team:
            return angle
        else:
            return math.atan2(-math.sin(angle), math.cos(angle))


    def on_device_ready(self, packet):
        self.team = packet.team


    def on_start(self, packet):
        self.team = packet.team


    def on_goto_finished(self, packet):
        self.pose = packet.current_pose


    def on_keep_alive(self, packet):
        self.pose = packet.current_pose


    def on_empty_tank_control(self, packet):
        self.tank_full = False
