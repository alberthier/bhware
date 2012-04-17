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
        self._team = TEAM_UNKNOWN
        self.event_loop = event_loop
        self.moving = False
        self.tank_full = False


    def is_looking_at(self, pose):
        dx = pose.x - self.pose.x
        dy = pose.y - self.pose.y
        angle = math.atan2(dy, dx) % (2.0 * math.pi)
        current_angle = self.pose.angle % (2.0 * math.pi)
        return abs(current_angle - angle) < (math.pi / 16.0)


    def is_looking_at_opposite(self, pose):
        dx = pose.x - self.pose.x
        dy = pose.y - self.pose.y
        angle = math.atan2(dy, dx) % (2.0 * math.pi)
        current_angle = (self.pose.angle + math.pi) % (2.0 * math.pi)
        return abs(current_angle - angle) < (math.pi / 16.0)


    def on_device_ready(self, packet):
        self.team = packet.team


    def on_start(self, packet):
        self.team = packet.team


    def on_goto_started(self, packet):
        self.moving = True


    def on_goto_finished(self, packet):
        self.moving = False
        self.pose = packet.current_pose


    def on_keep_alive(self, packet):
        self.pose = packet.current_pose


    def on_empty_tank_control(self, packet):
        self.tank_full = False


    def set_team(self, team):
        trajectory.Pose.match_team = team
        self._team = team


    def get_team(self):
        return self._team


    team = property(get_team, set_team)
