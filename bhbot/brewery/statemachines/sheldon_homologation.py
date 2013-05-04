# encoding: utf-8

import math
from math import pi

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
        yield CalibratePosition(0.65)
        yield MoveLineTo(0.65, 2.40)
        yield Rotate(0.0)
        yield MoveLineTo(0.95, 2.40)
        yield Rotate(math.pi/2.0)
        yield SendPacketsAndWaitAnswer(
            packet.Gripper(side = SIDE_LEFT, move = MOVE_OPEN),
            packet.Gripper(side = SIDE_RIGHT, move = MOVE_OPEN)
        )
        yield MoveLineTo(0.95, 0.5)
