#!/usr/bin/env python
# encoding: utf-8

import statemachine
import packets
import trajectory

from definitions import *




class DefaultStateMachine(statemachine.StateMachine):

    def __init__(self):
        statemachine.StateMachine.__init__(self, WaitDeviceReady)




class WaitDeviceReady(statemachine.State):

    def on_device_ready(self, team):
        self.switch_to_state(WaitStart)




class WaitStart(statemachine.State):

    def on_start(self, team):
        self.switch_to_state(Moving)




class Moving(statemachine.State):

    def on_enter(self):
        packet = packets.Goto()
        packet.movement = MOVEMENT_MOVE
        packet.direction = DIRECTION_FORWARD
        packet.points.append(trajectory.Pose(1000.0, 0.0))
        self.send_packet(packet)


    def on_goto_finished(self, reason):
        pass
