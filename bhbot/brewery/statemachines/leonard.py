# encoding: utf-8

import math

import statemachine
import packets
import position
import logger
import commonstates
import goalmanager

from definitions import *
from commonstates import *
from position import *



class Main(statemachine.State):

    def on_device_ready(self, packet):
        yield CalibratePosition()


    def on_start(self, packet):
        logger.log("Match started")
        yield TakeGlasses()
        yield EndOfMatch()




class TakeGlasses(statemachine.State):

    def on_enter(self):
        yield MoveLineTo(1.50,2.0)
        yield OpenGifts()


class OpenGifts(statemachine.State):
    X_VALUE = 1.83
    Y_VALUES = [1.7, 1.1, 0.5]
    def on_enter(self):
        self.gift_opener_side = GIFT_OPENER_POSITION_LEFT if self.robot.team == TEAM_BLUE else \
            GIFT_OPENER_POSITION_RIGHT
        yield MoveCurve(-math.pi/2, [(self.X_VALUE,2.3)])
        points = [ (self.X_VALUE, y) for y in self.Y_VALUES ]
        yield StateChain(self, MoveLine(points))

    def on_waypoint_reached(self, packet):
        self.send_packet(packets.GiftOpener(position=self.gift_opener_side))

    def on_gift_opener(self, packet):
        if packet.position != GIFT_OPENER_POSITION_IDLE:
            self.send_packet(packets.GiftOpener(position=GIFT_OPENER_POSITION_IDLE))

class EndOfMatch(statemachine.State):

    def on_enter(self):
        yield StopAll()

