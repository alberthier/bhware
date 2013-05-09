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
        yield CalibratePosition(START_X)
        yield Timer(500)
        detector = yield FetchCandleColors()

        # 1 photo
        yield MoveLineTo( START_X, 0.75)
        yield Rotate(0.0)


        # 2 photos
        #yield MoveLineTo( START_X, FIRST_LINE_END_Y)

        # Ajout

        #yield Rotate(math.pi)
        #shot_x = 2.0 - 0.51
        #yield MoveLineTo(shot_x, FIRST_LINE_END_Y, DIRECTION_BACKWARDS)
        #yield Rotate(math.radians(150))

        #######

        #yield Timer(200)
        #detector = yield FetchCandleColors()
        #yield Rotate(-math.pi / 2.0)
        #yield MoveLineTo(shot_x, 0.75)
        #yield Rotate(0.0)
        yield MoveLineTo(0.2, 0.75, DIRECTION_BACKWARDS)
        yield SpeedControl(0.2)
        yield MoveLineTo(0.0, 0.75, DIRECTION_BACKWARDS)
        yield SpeedControl(88.0)




class EndOfMatch(statemachine.State):

    def on_enter(self):
        yield StopAll();
        yield Pump(PUMP_ON)
        yield Timer(9000)
        yield Pump(PUMP_OFF)

