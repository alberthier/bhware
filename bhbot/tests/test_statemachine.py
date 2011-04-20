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


class TestSubState1(statemachine.State):

    def on_device_busy(self):
        self.finished_substate = None
        self.switch_to_substate(TestSubState2())


    def on_exit_substate(self, substate):
        self.finished_substate = substate.__class__.__name__





class TestSubState2(statemachine.State):

    def on_device_busy(self):
        self.finished_substate = None
        self.switch_to_substate(TestSubState3())


    def on_exit_substate(self, substate):
        self.finished_substate = substate.__class__.__name__


    def on_device_ready(self, team):
        self.exit_substate()





class TestSubState3(statemachine.State):

    def on_device_ready(self, team):
        self.exit_substate()




class SubStateStateTestCase(unittest.TestCase):

    def setUp(self):
        shared_data = {}
        self.root_state = statemachine.State()
        self.root_state.switch_to_substate(TestSubState1())


    def get_current_state(self):
        state = self.root_state
        while state.sub_state != None:
            state = state.sub_state
        return state


    def test_chain(self):
        self.assertTrue(isinstance(self.get_current_state(), TestSubState1))
        self.get_current_state().on_device_busy()
        self.assertTrue(isinstance(self.get_current_state(), TestSubState2))
        self.get_current_state().on_device_busy()
        self.assertTrue(isinstance(self.get_current_state(), TestSubState3))
        self.get_current_state().on_device_ready(TEAM_RED)
        self.assertTrue(isinstance(self.get_current_state(), TestSubState2))
        self.assertEqual(self.get_current_state().finished_substate, "TestSubState3")
        self.get_current_state().on_device_ready(TEAM_RED)
        self.assertTrue(isinstance(self.get_current_state(), TestSubState1))
        self.assertEqual(self.get_current_state().finished_substate, "TestSubState2")
