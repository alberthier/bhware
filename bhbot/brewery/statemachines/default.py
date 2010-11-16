#!/usr/bin/env python
# encoding: utf-8

import statemachine



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
        self.robot.send_packet(packets.Goto())


    def on_goto_finished(self, reason):
        pass
