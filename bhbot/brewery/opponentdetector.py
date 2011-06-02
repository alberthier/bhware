#!/usr/bin/env python
# encoding: utf-8


from definitions import *
import logging
import logger



class OpponentDetector(object):

    def __init__(self, event_loop):
        self.event_loop = event_loop
        self.detection_tick = 0


    def on_turret_detect(self, angle):
        # logging.debug("OpponentDetector.on_turret_detect {0}".format(angle))
        robot = self.event_loop.robot
        stop = False
        if robot.current_move_direction != None:
            if robot.current_move_direction == DIRECTION_FORWARD:
                stop = angle < OPPONENT_DETECTION_ANGLE or angle > 2.0 * math.pi - OPPONENT_DETECTION_ANGLE
            else:
                stop = angle > math.pi - OPPONENT_DETECTION_ANGLE - math.pi and angle < math.pi + OPPONENT_DETECTION_ANGLE

            if  stop:
                if self.detection_tick == 0:
                    self.detection_tick = OPPONENT_DETECTION_DISAPEARING_KEEP_ALIVE_TICKS
                    self.event_loop.get_current_state().on_opponent_entered(angle)
                self.event_loop.get_current_state().on_opponent_detected(angle)


    def on_keep_alive(self, current_pose, match_started, match_time):
        if self.detection_tick != 0:
            self.detection_tick -= 1
            if self.detection_tick == 0:
                self.event_loop.get_current_state().on_opponent_left()

