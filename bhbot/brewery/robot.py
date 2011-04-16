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


    def move(self, dx, dy):
        self.goto(self.pose.x + dx, self.pose.y + dy, None, DIRECTION_FORWARD)


    def move_to(self, x, y):
        self.goto(x, y, None, DIRECTION_FORWARD)


    def forward(self, distance):
        dx = math.cos(self.pose.angle) * distance
        dy = math.sin(self.pose.angle) * distance
        self.goto(self.pose.x + dx, self.pose.y + dy, None, DIRECTION_FORWARD)


    def backward(self, distance):
        dx = math.cos(self.pose.angle) * distance
        dy = math.sin(self.pose.angle) * distance
        self.goto(self.pose.x - dx, self.pose.y - dy, None, DIRECTION_BACKWARD)


    def look_at(self, x, y):
        dx = x - self.pose.x
        dy = y - self.pose.y
        self.rotate_to(math.atan2(dy, dx))


    def rotate(self, da):
        self.goto(None, None, self.pose.angle + da, DIRECTION_FORWARD)

    def look_at(self, x,y):
        """Rotate to look at position x,y"""
        pass


    def rotate_to(self, angle):
        self.goto(None, None, angle, DIRECTION_FORWARD)

    def goto_pose(self, pose) :
        if hasattr(pose,"change_team") : pose.change_team(self.team)
        return self.goto( *pose.get_values())

    def goto(self, x, y, angle, direction):
        packet = packets.Goto()
        if not tools.quasi_null(x) or not tools.quasi_null(y) :
            packet.movement = MOVEMENT_MOVE
        else:
            packet.movement = MOVEMENT_ROTATE
        # logger.log("Goto {0} {1} {2} {3} {4}".format(x,y,angle,direction,packet.movement))
        packet.direction = direction
        packet.points = [trajectory.Pose(x, y, angle)]
        self.event_loop.send_packet(packet)
