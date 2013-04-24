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

    def on_enter(self):
        statemachine.StateMachine(self.event_loop, "barman", side = SIDE_RIGHT)


    def on_device_ready(self, packet):
        yield CalibratePosition()
        yield AntiBlocking(True)


    def on_start(self, packet):
        self.log("Match started")
        yield TakeGlasses()
        yield OpenGifts()
        yield Deposit()
        yield EndOfMatch()




class Deposit(statemachine.State):

    def on_enter(self):
        yield MoveCurveTo(-1.77, (1.66, 0.38))
        yield DepositGlasses()
        yield None




class TakeGlasses(statemachine.State):

    def on_enter(self):
        yield MoveLineTo(BLUE_START_X, 2.0)
        yield None




class OpenGifts(statemachine.State):

    X_VALUE = 1.83
    Y_VALUES = [1.7, 1.1, 0.5]

    def on_enter(self):
        self.gift_opener_side = GIFT_OPENER_POSITION_LEFT if self.robot.team == TEAM_BLUE else \
            GIFT_OPENER_POSITION_RIGHT
        yield MoveCurve(-math.pi/2, [(self.X_VALUE,2.3)])
        points = [ (self.X_VALUE, y) for y in self.Y_VALUES ]
        move = MoveLine(points)
        move.on_waypoint_reached = self.on_waypoint_reached
        move.on_gift_opener = self.on_gift_opener
        yield move
        yield None


    def on_waypoint_reached(self, packet):
        self.send_packet(packets.GiftOpener(position=self.gift_opener_side))


    def on_gift_opener(self, packet):
        if packet.position != GIFT_OPENER_POSITION_IDLE:
            self.send_packet(packets.GiftOpener(position=GIFT_OPENER_POSITION_IDLE))




class EndOfMatch(statemachine.State):

    def on_enter(self):
        yield StopAll()

