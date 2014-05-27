# encoding: utf-8

import collections
import math

import statemachine
import packets
import position

from definitions import *
from commonstates import *
from position import *
from tools import *




class AvoidanceTest(statemachine.State):

    def __init__(self, start = None, end = None, navigate = False):
        self.navigate = navigate
        if IS_MAIN_ROBOT:
            self.start = (0.6, 0.25) if start is None else start
            self.end = (0.6, 2.0) if end is None else end
        else:
            self.start = (0.6, sym_y(0.25)) if start is None else start
            self.end = (0.6, 1.0) if end is None else end


    def on_enter(self):
        if IS_MAIN_ROBOT:
            yield Timer(2000)
        yield Navigate(*self.start)
        yield LookAt(*self.end)
        if IS_MAIN_ROBOT:
            yield Timer(2000)
            self.send_packet(packets.InterbotGeneric())
            yield from self.this_field_is_too_small_for_both_of_us()


    def on_interbot_generic(self, packet):
        yield from self.this_field_is_too_small_for_both_of_us()


    def this_field_is_too_small_for_both_of_us(self):
        yield SpeedControl(0.2)
        if self.navigate:
            move = Navigate(*self.end)
        else:
            move = MoveLineTo(*self.end)
        yield move
        if move.exit_reason == TRAJECTORY_DESTINATION_REACHED:
            self.log("Destination reached")
        elif move.exit_reason == TRAJECTORY_BLOCKED:
            self.log("Blocked")
        elif move.exit_reason == TRAJECTORY_OPPONENT_DETECTED:
            self.log("Opponent Detected")
        elif move.exit_reason == TRAJECTORY_DESTINATION_UNREACHABLE:
            self.log("Destination unreachable")
        yield None

