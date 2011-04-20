#!/usr/bin/env python
# encoding: utf-8

import unittest

import statemachine

from definitions import *


########################################################################




class TestStartState(statemachine.State):

    def __init__(self, shared_data):
        statemachine.State.__init__(self)
        self.state_data = None
        self.shared_data = shared_data

    def on_enter(self):
        self.state_data = "on enter"
        self.shared_data["key1"] = "value1"


    def on_device_ready(self, team):
        self.state_data = "device ready"


    def on_start(self, team):
        self.switch_to_state(TestEndState(self.shared_data))





class TestEndState(statemachine.State):

    def __init__(self, shared_data):
        statemachine.State.__init__(self)
        self.shared_data = shared_data


    def on_enter(self):
        self.shared_data["key1"] = "finished"





class StateMachineTestCase(unittest.TestCase):

    def setUp(self):
        shared_data = {}
        self.state = statemachine.State()
        self.state.switch_to_substate(TestStartState(shared_data))

    def test_init(self):
        self.assertTrue(isinstance(self.state.sub_state, TestStartState))
        self.assertEqual(self.state.sub_state.shared_data["key1"], "value1")
        self.assertEqual(self.state.sub_state.state_data, "on enter")

    def test_first_event(self):
        self.state.sub_state.on_device_ready(TEAM_RED)
        self.assertEqual(self.state.sub_state.state_data, "device ready")

    def test_transition(self):
        self.state.sub_state.on_device_ready(TEAM_RED)
        self.state.sub_state.on_start(TEAM_RED)
        self.assertTrue(isinstance(self.state.sub_state, TestEndState))
        self.assertEqual(self.state.sub_state.shared_data["key1"], "finished")


########################################################################

