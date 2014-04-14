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
        statemachine.StateMachine("opponentdetector", opponent_type = OPPONENT_ROBOT_MAIN)
        statemachine.StateMachine("opponentdetector", opponent_type = OPPONENT_ROBOT_SECONDARY)


    def on_device_ready(self, packet):
        yield AntiBlocking(True)
        yield CalibratePosition()


    def on_start(self, packet):
        logger.log("Starting ...")




##################################################
# End of match - Baloon




class EndOfMatch(statemachine.State):

    def on_enter(self):
        yield StopAll();
