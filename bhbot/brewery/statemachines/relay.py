# encoding: utf-8

import math

import statemachine
import packets
import commonstates

from definitions import *
from commonstates import *
from position import *




class Main(statemachine.State):

    def on_enter(self):
        self.current_toggles = {}


    def on_relay_toggling(self, packet):
        if packet.action == ACTION_ON:
            self.current_toggles[packet.id] = ACTION_ON
        elif packet.id in self.current_toggles:
            del self.current_toggles[packet.id]


    def on_timer_tick(self):
        for id in self.current_toggles.keys():
            action = ACTION_OFF if self.current_toggles[id] == ACTION_ON else ACTION_ON
            self.current_toggles[id] = action
            self.send_packet(packets.RelayControl(id = id, action = action))

