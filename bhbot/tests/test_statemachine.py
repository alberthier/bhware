#!/usr/bin/env python
# encoding: utf-8

import unittest

import statemachine

from definitions import *




class TestStateMachine(statemachine.StateMachine):

    def __init__(self):
        statemachine.StateMachine.__init__(self, TestStartState)
        self.global_data = "global data"





class TestStartState(statemachine.State):

    def __init__(self):
        statemachine.State.__init__(self)
        self.state_data = None

    def on_enter(self):
        self.state_data = "on enter"


    def on_device_ready(self, team):
        self.state_data = "device ready"


    def on_start(self, team):
        self.switch_to_state(TestEndState)





class TestEndState(statemachine.State):

    def __init__(self):
        statemachine.State.__init__(self)


    def on_enter(self):
        self.fsm.global_data = "finished"





class StateMachineTestCase(unittest.TestCase):

    def setUp(self):
        self.fsm = TestStateMachine()
        self.fsm.start()

    def test_init(self):
        self.assertTrue(isinstance(self.fsm.state, TestStartState))
        self.assertEqual(self.fsm.global_data, "global data")
        self.assertEqual(self.fsm.state.state_data, "on enter")

    def test_first_event(self):
        self.fsm.state.on_device_ready(TEAM_RED)
        self.assertEqual(self.fsm.state.state_data, "device ready")

    def test_transition(self):
        self.fsm.state.on_device_ready(TEAM_RED)
        self.fsm.state.on_start(TEAM_RED)
        self.assertTrue(isinstance(self.fsm.state, TestEndState))
        self.assertEqual(self.fsm.global_data, "finished")


