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
        self.walk = commonstates.TrajectoryWalk()
        #self.walk.wait_for(commonstates.CandleKicker(SIDE_RIGHT, CANDLE_KICKER_LOWER, CANDLE_KICKER_POSITION_UP))
        #self.walk.wait_for(commonstates.CandleKicker(SIDE_RIGHT, CANDLE_KICKER_LOWER, CANDLE_KICKER_POSITION_KICK))
        #self.walk.wait_for(commonstates.CandleKicker(SIDE_RIGHT, CANDLE_KICKER_LOWER, CANDLE_KICKER_POSITION_IDLE))
        #self.walk.wait_for(commonstates.GiftOpener(GIFT_OPENER_POSITION_LEFT))
        #self.walk.wait_for(commonstates.GiftOpener(GIFT_OPENER_POSITION_IDLE))
        #self.walk.wait_for(commonstates.GiftOpener(GIFT_OPENER_POSITION_RIGHT))
        self.walk.look_at(1.5, 1.6)
        self.walk.move_to(1.5, 1.6)
        self.walk.on_glass_present = self.on_glass_present
        self.switch_to_substate(self.walk)
        self.first = True


    def on_glass_present(self, packet):
        self.send_packet(packets.Stop())


    def on_exit_substate(self, substate):
        if self.first:
            self.first = False
            self.send_packet(packets.Nipper(side=SIDE_LEFT, move=MOVE_CLOSE))
            self.walk = commonstates.TrajectoryWalk()
            self.walk.look_at(1.0, 1.0)
            self.walk.move_to(1.0, 1.0)
            self.switch_to_substate(self.walk)



class EndOfMatch(statemachine.State):

    pass
