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




class EndOfMatch(statemachine.State):

    def on_enter(self):
        yield StopAll();

