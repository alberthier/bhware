#!/usr/bin/env python
# encoding: utf-8

import math

from definitions import *
import logging
import logger
import packets




class Opponent(object):

    IN_FRONT_ANGLES = [16, 17, 0, 1, 2]
    IN_BACK_ANGLES = [7, 8, 9, 10, 11]

    def __init__(self, detector, opponent_type):
        self.detector = detector
        self.opponent_type = opponent_type
        self.notified_in_front = False
        self.notified_in_back = False
        self.x = None
        self.y = None
        self.timeout_ticks = 0


    def on_turret_detect(self, packet):
        if packet.distance == 1:
            # Ignore long distances for the moment
            return
            distance = TURRET_LONG_DISTANCE_DETECTION_RANGE
        else:
            distance = TURRET_SHORT_DISTANCE_DETECTION_RANGE

        robot_pose = self.detector.event_loop.robot.pose
        real_angle = (packet.angle * 20.0 / 180.0) * math.pi
        real_angle += robot_pose.angle
        self.x = robot_pose.x + distance * math.cos(real_angle)
        self.y = robot_pose.y + distance * math.sin(real_angle)

        self.detector.event_loop.map.opponent_detected(packet.robot, self.x, self.y)

        if IS_HOST_DEVICE_PC:
            sim_packet = packets.SimulatorOpponentsPositions()
            sim_packet.robot = packet.robot
            sim_packet.present = True
            sim_packet.x = self.x
            sim_packet.y = self.y
            sim_packet.distance = distance
            self.detector.event_loop.send_packet(sim_packet)

        self.timeout_ticks = OPPONENT_DETECTION_DISAPEARING_TICKS

        is_in_front = packet.angle in self.IN_FRONT_ANGLES
        is_in_back  = packet.angle in self.IN_BACK_ANGLES
        self.notify_state_machine(packet, is_in_front, is_in_back)


    def on_timer_tick(self):
        if self.timeout_ticks != 0:
            self.timeout_ticks -= 1
            if self.timeout_ticks == 0:
                self.detector.event_loop.map.opponent_disapeared(self.opponent_type)
                if IS_HOST_DEVICE_PC:
                    sim_packet = packets.SimulatorOpponentsPositions()
                    sim_packet.robot = self.opponent_type
                    sim_packet.present = False
                    self.detector.event_loop.send_packet(sim_packet)
                self.notify_state_machine(None, False, False)


    def notify_state_machine(self, packet, is_in_front, is_in_back):
        if self.notified_in_front != is_in_front:
            if is_in_front:
                self.detector.event_loop.get_current_state().on_opponent_in_front(packet)
            else:
                self.detector.event_loop.get_current_state().on_opponent_disapeared(self.opponent_type, True)
            self.notified_in_front = is_in_front

        if self.notified_in_back != is_in_back:
            if is_in_back:
                self.detector.event_loop.get_current_state().on_opponent_in_back(packet)
            else:
                self.detector.event_loop.get_current_state().on_opponent_disapeared(self.opponent_type, False)
            self.notified_in_back = is_in_back




class OpponentDetector(object):

    def __init__(self, event_loop):
        self.event_loop = event_loop
        self.main_opponent = Opponent(self, OPPONENT_ROBOT_MAIN)
        self.secondary_opponent = Opponent(self, OPPONENT_ROBOT_SECONDARY)


    def on_turret_detect(self, packet):
        if packet.robot == OPPONENT_ROBOT_MAIN:
            self.main_opponent.on_turret_detect(packet)
        else:
            self.secondary_opponent.on_turret_detect(packet)


    def on_timer_tick(self):
        self.main_opponent.on_timer_tick()
        self.secondary_opponent.on_timer_tick()
