# encoding: utf-8

import math

from definitions import *
import logging
import logger
import packets
import eventloop




class Opponent(object):

    IN_FRONT_ANGLES = [16, 17, 0, 1, 2]
    IN_BACK_ANGLES = [7, 8, 9, 10, 11]

    def __init__(self, detector, opponent_type):
        self.detector = detector
        self.opponent_type = opponent_type
        self.opponent_direction = None
        self.x = None
        self.y = None
        self.timer = eventloop.Timer(self.detector.event_loop, OPPONENT_DETECTION_DISAPEARING_MS, self.opponent_disapear_timout)
        self.enabled = True


    def on_turret_detect(self, packet):
        if not self.enabled:
            return
        if packet.robot != self.opponent_type:
            return
        if packet.distance == 1:
            distance = TURRET_LONG_DISTANCE_DETECTION_RANGE
            return
        else:
            distance = TURRET_SHORT_DISTANCE_DETECTION_RANGE

        robot_pose = self.detector.event_loop.robot.pose
        real_angle = (packet.angle * 20.0 / 180.0) * math.pi
        real_angle += robot_pose.angle
        self.x = robot_pose.x + distance * math.cos(real_angle)
        self.y = robot_pose.y + distance * math.sin(real_angle)

        previous_direction = self.opponent_direction
        if packet.angle in self.IN_FRONT_ANGLES:
            self.opponent_direction = DIRECTION_FORWARDS
        elif packet.angle in self.IN_BACK_ANGLES:
            self.opponent_direction = DIRECTION_BACKWARDS
        else:
            self.opponent_direction = None

        if IS_HOST_DEVICE_PC:
            sim_packet = packets.SimulatorOpponentsPositions()
            sim_packet.robot = packet.robot
            sim_packet.present = True
            sim_packet.x = self.x
            sim_packet.y = self.y
            sim_packet.distance = distance
            self.detector.event_loop.send_packet(sim_packet)

        self.detector.event_loop.map.on_opponent_detected(packet, self.opponent_direction, self.x, self.y)

        if self.detector.event_loop.fsm is not None:
            if self.opponent_direction is None:
                self.detector.event_loop.fsm.on_opponent_disapeared(self.opponent_type, previous_direction)
            else:
                self.detector.event_loop.fsm.on_opponent_detected(packet, self.opponent_direction, self.x, self.y)

        self.timer.restart()


    def opponent_disapear_timout(self):
        previous_direction = self.opponent_direction
        self.opponent_direction = None
        self.detector.event_loop.map.on_opponent_disapeared(self.opponent_type, previous_direction)
        if IS_HOST_DEVICE_PC:
            sim_packet = packets.SimulatorOpponentsPositions(robot = self.opponent_type, present = False)
            self.detector.event_loop.send_packet(sim_packet)
        if self.detector.event_loop.fsm is not None:
            self.detector.event_loop.fsm.on_opponent_disapeared(self.opponent_type, previous_direction)




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


    def enable(self):
        logger.log("OpponentDetector: enabling")
        self.main_opponent.enabled = True
        self.secondary_opponent.enabled = True
