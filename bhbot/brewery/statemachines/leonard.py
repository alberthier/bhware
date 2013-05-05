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

X_OPEN_GIFTS_START = 1.83
Y_OPEN_GIFTS_START = 2.38

X_START = BLUE_START_X
Y_START = BLUE_START_Y


class Main(statemachine.State):

    def on_enter(self):
        statemachine.StateMachine(self.event_loop, "barman", side = SIDE_RIGHT)


    def on_device_ready(self, packet):
        yield CalibratePosition(X_START)
        yield AntiBlocking(True)


    def on_start(self, packet):
        yield TakeGlasses()
        yield LookAt(X_OPEN_GIFTS_START, Y_OPEN_GIFTS_START)

        yield OpenGifts()
        yield Deposit()
        yield EndOfMatch()




class Deposit(statemachine.State):

    def on_enter(self):
        yield LookAt(1.66, 0.38)
        yield MoveLineTo(1.66, 0.38)
        yield DepositGlasses()
        yield None




class TakeGlasses(statemachine.State):

    def on_enter(self):
        yield MoveLineTo(BLUE_START_X, Y_OPEN_GIFTS_START)
        yield None



class GotoGiftOpenStart(statemachine.State):

    def on_enter(self):
        yield MoveLineTo(X_OPEN_GIFTS_START, Y_OPEN_GIFTS_START)
        yield None

class OpenGifts(statemachine.State):

    X_VALUE = X_OPEN_GIFTS_START
    Y_VALUES = [Y_OPEN_GIFTS_START, 1.79, 1.2, 0.62]

    def on_enter(self):
        self.gift_opener_side = GIFT_OPENER_POSITION_RIGHT \
            if self.robot.team == TEAM_BLUE else GIFT_OPENER_POSITION_LEFT

        yield Rotate(0.0)
        yield GotoGiftOpenStart()
        yield Rotate(math.pi/2)
        # yield GiftOpener(self.gift_opener_side)
        # yield GiftOpener(GIFT_OPENER_POSITION_IDLE)

        points = [ (self.X_VALUE, y) for y in self.Y_VALUES ]
        move = MoveLine(points, direction=DIRECTION_BACKWARDS)
        move.on_waypoint_reached = self.on_waypoint_reached
        move.on_gift_opener = self.on_gift_opener
        yield move
        yield GiftOpener(self.gift_opener_side)
        yield GiftOpener(GIFT_OPENER_POSITION_IDLE)
        yield None


    def on_waypoint_reached(self, packet):
        self.send_packet(packets.GiftOpener(position=self.gift_opener_side))


    def on_gift_opener(self, packet):
        if packet.position != GIFT_OPENER_POSITION_IDLE:
            self.send_packet(packets.GiftOpener(position=GIFT_OPENER_POSITION_IDLE))




class EndOfMatch(statemachine.State):

    def on_enter(self):
        yield StopAll()

