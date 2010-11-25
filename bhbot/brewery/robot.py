#!/usr/bin/env python
# encoding: utf-8

import trajectory
import packets

from definitions import *




class Robot(object):

    def __init__(self, event_loop):
        self.pose = trajectory.Pose(0.0, 0.0, 0.0)
        self.team = TEAM_UNKNOWN
        self.event_loop = event_loop


    def move(self, dx, dy):
        self.goto(self.pose.x + dx, self.pose.y, None)

    def move_to(self, x, y):
        self.goto(x, y, None)

    def rotate(self, dangle):
        self.goto(None, None, self.pose.angle + dangle)

    def rotate_to(self, angle):
        self.goto(None, None, angle)

    def goto(self, x, y, angle):
        packet = packets.Goto()
        if angle == None:
            packet.movement = MOVEMENT_MOVE
        else:
            packet.movement = MOVEMENT_ROTATE
        packet.direction = DIRECTION_FORWARD
        packet.points = [trajectory.Pose(x, y, angle)]
        self.event_loop.send_packet(packet)
