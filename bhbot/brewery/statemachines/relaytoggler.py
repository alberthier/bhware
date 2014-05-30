# encoding: utf-8

import math

import statemachine
import packets
import commonstates

from definitions import *
from commonstates import *
from position import *




class Main(statemachine.State):

    ACTION = 0
    COUNT  = 1

    def on_enter(self):
        self.current_toggles = {}
        self.count = 0


    def on_relay_toggle(self, packet):
        if packet.toggle_count == FLIP_FLOP_MODE:
            # in flip-flop mode, directly add or remove the relay to toggle to the list
            if packet.action == ACTION_ON:
                self.current_toggles[packet.id] = (packet.action, packet.toggle_count)
            elif packet.id in self.current_toggles:
                del self.current_toggles[packet.id]
            self.send_packet(packets.RelayToggle(packet.id, packet.action, packet.toggle_count))
        else:
            # in classic mode, store the relay to toggle, with the action or the opposite action
            # depending on the fact that the number of toggles is even or odd
            if packet.toggle_count % 2 == 0:
                action = packet.action
            else:
                action = self.reverse(packet.action)
            self.current_toggles[packet.id] = (action, packet.toggle_count)


    def on_timer_tick(self):
        self.count += 1
        if self.count == 2:
            self.count = 0
            keys_to_remove = []
            for id in self.current_toggles.keys():
                action = self.reverse(self.current_toggles[id][self.ACTION])
                count = self.current_toggles[id][self.COUNT]
                if count != FLIP_FLOP_MODE:
                    count -= 1
                if count != 0:
                    self.current_toggles[id] = (action, count)
                else:
                    keys_to_remove.append(id)
                self.send_packet(packets.RelayControl(id = id, action = action))
            for id in keys_to_remove:
                self.send_packet(packets.RelayToggle(id, self.current_toggles[id][self.ACTION], 0))
                del self.current_toggles[id]


    def reverse(self, action):
        return ACTION_ON if action == ACTION_OFF else ACTION_OFF

