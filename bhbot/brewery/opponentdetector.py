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
        logger.log("OpponentDetector.on_turret_detect {0}".format(angle))
        robot = self.event_loop.robot
        stop = False
        if robot.current_move_direction != None:
            logger.log("OpponentDetector robot.current_move_direction={0}".format(robot.current_move_direction))
            if robot.current_move_direction == DIRECTION_FORWARD:
                stop = angle < OPPONENT_DETECTION_ANGLE or angle > 2.0 * math.pi - OPPONENT_DETECTION_ANGLE
                logger.log("OpponentDetector robot.current_move_direction=FORWARD stop={0}".format(stop))
            else:
                stop = angle > math.pi - OPPONENT_DETECTION_ANGLE - math.pi and angle < math.pi + OPPONENT_DETECTION_ANGLE
                logger.log("OpponentDetector robot.current_move_direction=FORWARD stop={0}".format(stop))

            if  stop:
                logger.log("OpponentDetector stop={0}".format(stop))
                if self.detection_tick == 0:
                    self.detection_tick = OPPONENT_DETECTION_DISAPEARING_KEEP_ALIVE_TICKS
                    logger.log("OpponentDetector call on_opponent_entered tick={0}".format(self.detection_tick))
                    self.event_loop.get_current_state().on_opponent_entered(angle)
                self.event_loop.get_current_state().on_opponent_detected(angle)
                logger.log("OpponentDetector call on_opponent_detected tick={0}".format(self.detection_tick))


    def on_keep_alive(self, current_pose, match_started, match_time):
        if self.detection_tick != 0:
            self.detection_tick -= 1
            if self.detection_tick == 0:
                self.event_loop.get_current_state().on_opponent_left()

