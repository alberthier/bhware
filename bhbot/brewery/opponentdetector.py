# encoding: utf-8

import math

import logging
import logger
import packets
import eventloop

from definitions import *




class Opponent(object):

    IN_FRONT_ANGLES = [16, 17, 0, 1, 2]
    IN_BACK_ANGLES = [7, 8, 9, 10, 11]

    def __init__(self, detector, opponent_type):
        self.detector = detector
        self.opponent_type = opponent_type
        self.opponent_direction = None
        self.x = None
        self.y = None
        self.timer = eventloop.Timer(self.detector.event_loop, OPPONENT_DETECTION_DISAPEARING_MS, self.opponent_disappeared)
        self.enabled = True
        self.detected = False


    def on_turret_detect(self, packet):
        if not self.enabled:
            return
        if packet.robot != self.opponent_type:
            return
        if packet.distance == 1:
            return
        else:
            distance = TURRET_SHORT_DISTANCE_DETECTION_RANGE

        angle = (18 - packet.angle) % 18
        angle = (angle - OpponentDetector.OFFSET) % 18

        robot_pose = self.detector.event_loop.robot.pose
        real_angle = (angle * 20.0 / 180.0) * math.pi
        real_angle += robot_pose.angle
        self.x = robot_pose.x + distance * math.cos(real_angle)
        self.y = robot_pose.y + distance * math.sin(real_angle)

        previous_direction = self.opponent_direction
        if packet.distance == 0:
            if angle in self.IN_FRONT_ANGLES:
                self.opponent_direction = DIRECTION_FORWARDS
            elif angle in self.IN_BACK_ANGLES:
                self.opponent_direction = DIRECTION_BACKWARDS
            else:
                self.opponent_direction = None
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

        self.detector.event_loop.send_packet(packets.OpponentPosition(robot = self.opponent_type, x = self.x, y = self.y))

        if self.opponent_direction is not None:
            if previous_direction is None:
                logger.log("Opponent detected")
                self.detected = True
                self.detector.event_loop.send_packet(packets.OpponentDetected(robot = self.opponent_type, direction = self.opponent_direction, x = self.x, y = self.y))
            self.timer.restart()
        elif self.opponent_direction is None and previous_direction is not None:
            self.opponent_disappeared()


    def opponent_disappeared(self):
        logger.log("Opponent disapeared")
        self.detected = False
        self.timer.stop()
        previous_direction = self.opponent_direction
        self.opponent_direction = None
        self.detector.event_loop.send_packet(packets.OpponentPosition(robot = self.opponent_type, x = None, y = None))
        if IS_HOST_DEVICE_PC:
            sim_packet = packets.SimulatorOpponentsPositions(robot = self.opponent_type, present = False)
            self.detector.event_loop.send_packet(sim_packet)
        self.detector.event_loop.send_packet(packets.OpponentDisappeared(robot = self.opponent_type, direction = self.opponent_direction))




class OpponentDetector(object):

    OFFSET = 0

    def __init__(self, event_loop):
        if IS_MAIN_ROBOT:
            OpponentDetector.OFFSET = 2
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
