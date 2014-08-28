# encoding: utf-8

import collections
import datetime
import math

import statemachine
import packets
import position

from definitions import *
from commonstates import *
from position import *
from tools import *

class CreateTurretResponseCurve(statemachine.State):

    def on_enter(self):
        dists = []
        for i in range(25):
            dist = i * 10
            yield SetupTurret(dist, dist)
            yield InitPosition()
            yield BackwardsUntilNoDetection()
            yield ForwardUntilDetection()
            self.log("dists[{}] = {}".format(dist, self.robot.pose.x))
            dists.append(self.robot.pose.x)






class InitPosition(statemachine.State):

    def on_enter(self):
        yield DefinePosition(3.0, 0.0, 0.0)
        yield MoveLineTo(0.0, 0.0)
        yield DefinePosition(ROBOT_X_SIZE, 0.0)
        yield None




class BackwardsUntilNoDetection(statemachine.State):

    def on_enter(self):
        move = MoveLineTo(3.0, 0.0, DIRECTION_BACKWARDS)
        move.on_turret_detect = self.on_turret_detect
        self.lastdetect = datetime.datetime.now()
        yield move


    def on_turret_detect(self):
        self.lastdetect = datetime.datetime.now()


    def on_timer_tick(self):
        if (datetime.datetime.now() - self.lastdetect > datetime.timedelta(0, 1, 0)):
            self.log("No more detections")
            yield None




class ForwardUntilDetection(statemachine.State):

    def on_enter(self):
        move = MoveLineTo(3.0, 0.0, DIRECTION_BACKWARDS)
        move.on_turret_detect = self.on_turret_detect
        yield mode


    def on_turret_detect(self):
        self.log("Detecting again")
        yield None
