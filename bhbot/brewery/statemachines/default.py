#!/usr/bin/env python
# encoding: utf-8

import math

import statemachine
import packets
import trajectory
import logger

from definitions import *




class DefaultStateMachine(statemachine.StateMachine):

    def __init__(self):
        statemachine.StateMachine.__init__(self, WaitDeviceReady)




class WaitDeviceReady(statemachine.State):

    def on_device_ready(self, team):
        self.switch_to_state(WaitStart)




class WaitStart(statemachine.State):

    def on_start(self, team):
        self.switch_to_state(WaitFirstKeepAlive)



class WaitFirstKeepAlive(statemachine.State):

    def on_keep_alive(self, current_pose, match_started, match_time):
        self.switch_to_state(GotoFieldMove)



class GotoFieldMove(statemachine.State):

    def on_enter(self):
        self.robot().forward(0.4)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(GotoFieldRotate)



class GotoFieldRotate(statemachine.State):

    def on_enter(self):
        angle = math.pi / 6.0
        if self.robot().team == TEAM_RED:
            angle = -angle
        self.robot().rotate(angle)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(GotoFieldMove2)



class GotoFieldMove2(statemachine.State):

    def on_enter(self):
        self.robot().forward(0.4)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(GotoFieldRotate2)



class GotoFieldRotate2(statemachine.State):

    def on_enter(self):
        angle = math.pi / 3.0
        if self.robot().team == TEAM_RED:
            angle = -angle
        self.robot().rotate(angle)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(Moving)



class Rotate(statemachine.State):

    def on_enter(self):
        angle = math.pi / 2.0
        if self.robot().team == TEAM_RED:
            angle = -angle
        self.robot().rotate(angle)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(Moving)


class Moving(statemachine.State):

    def on_enter(self):
        self.robot().forward(0.3)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(Rotate)
