#!/usr/bin/env python
# encoding: utf-8

import statemachine



class DefaultStateMachine(statemachine.StateMachine):

    def __init__(self):
        statemachine.StateMachine.__init__(self, WaitDeviceReady)




class WaitDeviceReady(statemachine.State):

    def __init__(self):
        statemachine.State.__init__(self)
