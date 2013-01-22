# encoding: utf-8

import math

import statemachine
import packets
import position
import logger
import commonstates
import goalmanager

from definitions import *




class Main(statemachine.State):

    def on_device_ready(self, packet):
        self.switch_to_substate(commonstates.DefinePosition())


    def on_exit_substate(self, substate):
        self.switch_to_state(WaitStart())




class WaitStart(statemachine.State):

    def on_start(self, packet):
        self.switch_to_substate(commonstates.DefinePosition())


    def on_exit_substate(self, substate):
        self.switch_to_state(Test1())




class Test1(statemachine.State):

    def on_enter(self):
        walk = commonstates.TrajectoryWalk()
        walk.look_at(1.8, 1.8)
        walk.move_to(1.8, 1.8)
        self.switch_to_substate(walk)



class EndOfMatch(statemachine.State):

    pass
