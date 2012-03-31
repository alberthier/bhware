#!/usr/bin/env python
# encoding: utf-8

import math

from definitions import *
import logging
import logger
import packets




class OpponentDetector(object):

    IN_FRONT_ANGLES = [16, 17, 0, 1, 2]
    IN_BACK_ANGLES = [7, 8, 9, 10, 11]

    def __init__(self, event_loop):
        self.event_loop = event_loop
        self.main_short_distance_ticks = 0
        self.main_long_distance_ticks = 0
        self.secondary_short_distance_ticks = 0
        self.secondary_long_distance_ticks = 0


    def on_turret_detect(self, packet):
        if packet.distance == 1:
            distance = TURRET_LONG_DISTANCE_DETECTION_RANGE
            if packet.robot == OPPONENT_ROBOT_MAIN:
                self.main_long_distance_ticks = OPPONENT_DETECTION_DISAPEARING_TICKS
            else:
                self.secondary_long_distance_ticks = OPPONENT_DETECTION_DISAPEARING_TICKS
        else:
            distance = TURRET_SHORT_DISTANCE_DETECTION_RANGE
            if packet.robot == OPPONENT_ROBOT_MAIN:
                self.main_short_distance_ticks = OPPONENT_DETECTION_DISAPEARING_TICKS
            else:
                self.secondary_short_distance_ticks = OPPONENT_DETECTION_DISAPEARING_TICKS

        robot_pose = self.event_loop.robot.pose
        real_angle = (packet.angle * 20.0 / 180.0) * math.pi
        real_angle += robot_pose.angle
        x = robot_pose.x + distance * math.cos(real_angle)
        y = robot_pose.y + distance * math.sin(real_angle)
        self.event_loop.map.opponent_detected(packet.robot, x, y)

        if IS_HOST_DEVICE_PC:
            sim_packet = packets.SimulatorOpponentsPositions()
            sim_packet.robot = packet.robot
            sim_packet.x = x
            sim_packet.y = y
            sim_packet.distance = distance
            self.event_loop.send_packet(sim_packet)

        if packet.distance == 0:
            # Opponent is in our way, notify the state machine
            if packet.angle in OpponentDetector.IN_FRONT_ANGLES:
                self.event_loop.get_current_state().on_opponent_in_front(packet)
            elif packet.angle in OpponentDetector.IN_BACK_ANGLES:
                self.event_loop.get_current_state().on_opponent_in_back(packet)


    def on_timer_tick(self):
        disapeared_opponents = set()
        if self.main_short_distance_ticks != 0:
            self.main_short_distance_ticks -= 1
            if self.main_short_distance_ticks == 0:
                disapeared_opponents.add(OPPONENT_ROBOT_MAIN)
                self.event_loop.get_current_state().on_opponent_disapeared(OPPONENT_ROBOT_MAIN)
        if self.main_long_distance_ticks != 0:
            self.main_long_distance_ticks -= 1
            if self.main_long_distance_ticks == 0:
                disapeared_opponents.add(OPPONENT_ROBOT_MAIN)
        if self.secondary_short_distance_ticks != 0:
            self.secondary_short_distance_ticks -= 1
            if self.secondary_short_distance_ticks == 0:
                disapeared_opponents.add(OPPONENT_ROBOT_SECONDARY)
                self.event_loop.get_current_state().on_opponent_disapeared(OPPONENT_ROBOT_SECONDARY)
        if self.secondary_long_distance_ticks != 0:
            self.secondary_long_distance_ticks -= 1
            if self.secondary_long_distance_ticks == 0:
                disapeared_opponents.add(OPPONENT_ROBOT_SECONDARY)

        for opponent in disapeared_opponents:
            self.event_loop.map.opponent_disapeared(opponent)
