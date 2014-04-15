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
        statemachine.StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_MAIN)
        statemachine.StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_SECONDARY)


    def on_device_ready(self, packet):
        yield AntiBlocking(True)

        yield Trigger(GUN_LOAD, PAINT_1_HOLD, PAINT_2_HOLD, FIRE_FLIPPER_CLOSE)

        yield CalibratePosition()


    def on_start(self, packet):
        self.yield_at(90000, EndOfMatch())
        logger.log("Starting ...")




##################################################
# End of match




class EndOfMatch(statemachine.State):

    def on_enter(self):
        yield StopAll();
