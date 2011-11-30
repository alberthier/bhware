#!/usr/bin/env python
# encoding: utf-8

import math

import statemachine
import packets
import trajectory
import logger
import commonstates
from definitions import *



################################################################################
# Setup states


class Main(statemachine.State):

    def on_device_ready(self, packet):
        self.switch_to_substate(commonstates.DefinePosition())


    def on_exit_substate(self, substate):
        self.switch_to_state(WaitStart())




class WaitStart(statemachine.State):

    def on_start(self, packet):
        self.switch_to_substate(commonstates.DefinePosition())


    def on_exit_substate(self, substate):
        self.switch_to_state(WaitFirstKeepAlive())




class WaitFirstKeepAlive(statemachine.State):

    def on_keep_alive(self, packet):
        self.switch_to_state(GotoStartPoint())




class GotoStartPoint(statemachine.State):

    def on_enter(self):
        self.walk = commonstates.TrajectoryWalk()
        self.walk.move_to(PURPLE_START_X, 0.59)
        if self.robot().team == TEAM_PURPLE:
            self.walk.rotate_to(0.0)
            self.walk.move_to(0.96, 0.59)
        else:
            self.walk.rotate_to(math.pi)
            self.walk.move_to(1.04, 0.59)
        self.switch_to_substate(self.walk)


    def on_exit_substate(self, substate):
        self.switch_to_state(LoopAroundPeanutIsland())




class LoopAroundPeanutIsland(statemachine.State):

    def on_enter(self):
        points = [(1.50, 1.06,  math.pi / 2.0),
                  (1.40, 1.46,  math.pi / 2.0),
                  (1.50, 1.86,  math.pi / 2.0),
                  (1.04, 2.41,  math.pi      ),
                  (0.50, 1.86, -math.pi / 2.0),
                  (0.60, 1.46, -math.pi / 2.0),
                  (0.50, 1.06, -math.pi / 2.0),
                  (0.96, 0.59,  0.0          )]

        if self.robot().team == TEAM_RED:
            points.reverse()
            points = points[1:] + points[:1]
            angle_offset = math.pi
        else:
            angle_offset = 0.0

        self.walk = commonstates.TrajectoryWalk()

        for i in xrange(len(points) * 3):
            k = i % len(points)
            (x, y, angle) = points[k]
            self.walk.goto(x, y, angle + angle_offset)

        self.switch_to_substate(self.walk)


    def on_exit_substate(self, substate):
        self.switch_to_state(GotoCaptainRoom())




class GotoCaptainRoom(statemachine.State):

    def on_enter(self):
        self.walk = commonstates.TrajectoryWalk()
        self.walk.move_to(PURPLE_START_X, 0.59)
        self.walk.rotate_to(PURPLE_START_ANGLE)
        self.walk.move_to(PURPLE_START_X, PURPLE_START_Y)
        self.switch_to_substate(self.walk)
