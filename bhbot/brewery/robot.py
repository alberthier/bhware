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


    def move(self, dx, dy, reference_team = TEAM_UNKNOWN):
        pose = self.get_reference_pose(reference_team)
        pose.x += dx
        pose.y += dy
        pose = self.get_real_pose(pose, reference_team)
        return self.goto(pose.x + dx, pose.y + dy, None, DIRECTION_FORWARD)


    def move_to(self, x, y, reference_team = TEAM_UNKNOWN):
        pose = self.get_reference_pose(reference_team)
        pose.x = x
        pose.y = y
        pose = self.get_real_pose(pose, reference_team)
        return self.goto(pose.x, pose.y, None, DIRECTION_FORWARD)


    def forward(self, distance, reference_team = TEAM_UNKNOWN):
        dx = math.cos(self.pose.angle) * distance
        dy = math.sin(self.pose.angle) * distance
        return self.move(dx, dy, None, DIRECTION_FORWARD)


    def backward(self, distance, reference_team = TEAM_UNKNOWN):
        dx = math.cos(self.pose.angle) * distance
        dy = math.sin(self.pose.angle) * distance
        return self.move(-dx, -dy, None, DIRECTION_BACKWARD)


    def look_at(self, x, y, reference_team = TEAM_UNKNOWN):
        pose = self.get_reference_pose(reference_team)
        dx = x - pose.x
        dy = y - pose.y
        pose.angle = math.atan2(dy, dx)
        pose = self.get_real_pose(pose, reference_team)
        return self.rotate_to(pose.angle)


    def rotate(self, da, reference_team = TEAM_UNKNOWN):
        pose = self.get_reference_pose(reference_team)
        pose.angle += da
        pose = self.get_real_pose(pose, reference_team)
        return self.goto(None, None, pose.angle, DIRECTION_FORWARD)


    def rotate_to(self, angle, reference_team = TEAM_UNKNOWN):
        pose = self.get_reference_pose(reference_team)
        pose.angle = angle
        pose = self.get_real_pose(pose, reference_team)
        return self.goto(None, None, angle, DIRECTION_FORWARD)


    def goto(self, x, y, angle, direction, reference_team = TEAM_UNKNOWN):
        pose = trajectory.Pose(x, y, angle)
        pose = self.get_real_pose(pose, reference_team)
        packet = packets.Goto()
        packet.movement = None
        if pose.x == None or tools.quasi_equal(pose.x, self.pose.x):
            pose.x = self.pose.x
        else:
            packet.movement = MOVEMENT_LINE
        if pose.y == None or tools.quasi_equal(pose.y, self.pose.y):
            pose.y = self.pose.y
        else:
            packet.movement = MOVEMENT_LINE
        if pose.angle == None or tools.quasi_equal(pose.angle, self.pose.angle):
            pose.angle = self.pose.angle
            if packet.movement == None:
                self.event_loop.inject_goto_finished()
                return None
        elif packet.movement == MOVEMENT_LINE:
            packet.movement = MOVEMENT_MOVE
        else:
            packet.movement = MOVEMENT_ROTATE

        packet.direction = direction
        packet.points = [trajectory.Pose(pose.x, pose.y, pose.angle)]
        self.event_loop.send_packet(packet)
        return packet
