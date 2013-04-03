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
        yield CalibratePosition()

    def on_device_ready(self, packet):
        logger.log("hello")

    def on_start(self, packet):
        move = yield Navigate(1.5, 2.4)
        logger.log("elapsed: " + str(self.event_loop.get_elapsed_match_time()))
        logger.log(move.exit_reason)




class EndOfMatch(statemachine.State):

    def on_enter(self):
        yield StopAll();
        yield Pump(PUMP_ON)
        yield Timer(9000)
        yield Pump(PUMP_OFF)

