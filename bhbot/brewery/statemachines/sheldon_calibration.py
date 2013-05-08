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
from statemachines.sheldon import *




class Main(statemachine.State):

    def on_device_ready(self, packet):
        yield AntiBlocking(True)


    def on_start(self, packet):
        for i in range(10):
            yield CalibratePosition(START_X)
            detector = yield FetchCandleColors()
            self.log("#{}[1] candles detection: {}".format(i, detector.colors))
            yield MoveLineTo( START_X, FIRST_LINE_END_Y)
            yield Rotate(SECOND_SHOT_ANGLE)
            detector = yield FetchCandleColors()
            self.log("#{}[2] candles detection: {}".format(i, detector.colors))
            yield Rotate(-math.pi / 2.0)
            yield MoveLineTo(START_X, 0.75)
            yield Rotate(0.0)
            yield MoveLineTo(0.0, 0.75)
            yield Timer(3000)




class EndOfMatch(statemachine.State):

    def on_enter(self):
        yield StopAll();
        yield Pump(PUMP_ON)
        yield Timer(9000)
        yield Pump(PUMP_OFF)

