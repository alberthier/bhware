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

    def on_device_ready(self, packet):
        #yield CalibratePosition(X_START)
        yield DefinePosition(X_START, ROBOT_CENTER_X, math.pi/2)
        yield AntiBlocking(True)


    def on_start(self, packet):
        gm = self.robot.goal_manager
        yield Gripper(SIDE_RIGHT, MOVE_OPEN)
        yield TakeGlasses()
        yield Gripper(SIDE_RIGHT, MOVE_CLOSE)
        yield LookAt(X_START, Y_START)
        yield MoveLineTo(X_START, 0.3)
        yield Gripper(SIDE_RIGHT, MOVE_OPEN)
        yield MoveRelative(-0.2, direction=DIRECTION_BACKWARDS)
        yield EndOfMatch()



class TakeGlasses(statemachine.State):

    def on_enter(self):
        yield MoveLineTo(BLUE_START_X, 2.0)
        yield None



class EndOfMatch(statemachine.State):

    def on_enter(self):
        yield StopAll()

