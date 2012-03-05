#!/usr/bin/env python
# encoding: utf-8


from definitions import *
import logging
import logger



class OpponentDetector(object):

    def __init__(self, event_loop):
        self.event_loop = event_loop
        self.detection_tick = 0


    def on_turret_detect(self, packet):
        logger.log("OpponentDetector.on_turret_detect {}".format(packet.angle))
        if self.is_opponent_in_front(packet.angle) or self.is_opponent_in_back(packet.angle):
            if self.detection_tick == 0:
                self.detection_tick = OPPONENT_DETECTION_DISAPEARING_TICKS
                self.event_loop.get_current_state().on_opponent_entered(packet.angle)
            self.event_loop.get_current_state().on_opponent_detected(packet.angle)


    def on_timer_tick(self):
        if self.detection_tick != 0:
            self.detection_tick -= 1
            if self.detection_tick == 0:
                self.event_loop.get_current_state().on_opponent_left()


    def is_opponent_in_front(self, angle):
        return angle < OPPONENT_DETECTION_ANGLE or angle > 2.0 * math.pi - OPPONENT_DETECTION_ANGLE


    def is_opponent_in_back(self, angle):
        return angle > math.pi - OPPONENT_DETECTION_ANGLE and angle < math.pi + OPPONENT_DETECTION_ANGLE
