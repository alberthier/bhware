# encoding: utf-8

import unittest
import run_tests
run_tests.patch_pythonpath()

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
        self.state_history = []
        self.state = statemachine.State()
        self.state.event_loop = self
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
        self.state_history = []
        self.root_state = statemachine.State()
        self.root_state.event_loop = self
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

class DummyPacket(object):
    def __init__(self) :
        self.METHOD_NAME = "on_keep_alive"

    def dispatch(self, obj):
        return obj.on_keep_alive()

class FsmLogger(object):
    def __init__(self):
        self.lines = []

    def log(self, s):
        print("Log : "+s)
        self.lines.append(s)

    def to_string(self):
        return "\n".join(self.lines)


def trim(s):
    s = s.strip()
    s = "\n".join((l.strip() for l in s.splitlines()))
    return s


class TestNewStateMachine(unittest.TestCase):
    def test_simple_state_switch(self):
        class State2(statemachine.State):
            def __init__(self):
                super(State2, self).__init__()

            def on_keep_alive(self):
                self.log("State2 A")
                yield State1()
                self.log("State2 B")

        class State1(statemachine.State):
            def __init__(self):
                super(State1, self).__init__()

            def on_keep_alive(self):
                self.log("State1 A")
                yield State2()
                self.log("State1 B")

        logger = FsmLogger()

        start_state = State1()
        fsm = statemachine.StateMachine(start_state)
        fsm.logger = logger

        fsm.dispatch(DummyPacket())

        self.assertEquals(State2, fsm.current_state.__class__)

        self.assertEqual(trim(
                            """
                            State1 A
                            """
                        ),
                         logger.to_string()
        )

    def test_back_and_forth(self):
        class State2(statemachine.State):
            def __init__(self):
                super(State2, self).__init__()

            def on_keep_alive(self):
                self.log("State2 A")
                yield State1()
                self.log("State2 B")

        class State1(statemachine.State):
            def __init__(self):
                super(State1, self).__init__()

            def on_keep_alive(self):
                self.log("State1 A")
                yield State2()
                self.log("State1 B")


        logger = FsmLogger()

        start_state = State1()
        fsm = statemachine.StateMachine(start_state)

        fsm.logger = logger

        fsm.dispatch(DummyPacket())
        fsm.dispatch(DummyPacket())

        self.assertEquals(State1, fsm.current_state.__class__)

        self.assertEqual(trim(
                            """
                            State1 A
                            State2 A
                            """
                        ),
                        logger.to_string()
        )

    def test_substate(self):
        class State2(statemachine.State):
            def __init__(self):
                super(State2, self).__init__()

            def on_keep_alive(self):
                self.log("State2")
                yield None

        class State1(statemachine.State):
            def __init__(self):
                super(State1, self).__init__()

            def on_keep_alive(self):
                self.log("State1 A")
                yield State2()
                self.log("State1 B")


        logger = FsmLogger()

        start_state = State1()
        fsm = statemachine.StateMachine(start_state)

        fsm.logger = logger

        fsm.dispatch(DummyPacket())
        fsm.dispatch(DummyPacket())

        self.assertIsInstance(fsm.current_state, State1)

        self.assertEqual(trim(
            """
            State1 A
            State2
            State1 B
            """
        ),
                         logger.to_string()
        )

    def test_substate_result(self):
        class State2(statemachine.State):
            def __init__(self):
                super(State2, self).__init__()
                self.result = 0

            def on_keep_alive(self):
                self.log("State2")
                self.result = 1
                yield None

        class State1(statemachine.State):
            def __init__(self):
                super(State1, self).__init__()
                self.check = 0

            def on_keep_alive(self):
                self.log("State1 A")
                state = yield State2()
                self.log("State1 B")
                self.check = state.result

        logger = FsmLogger()

        start_state = State1()
        fsm = statemachine.StateMachine(start_state)

        fsm.logger = logger

        fsm.dispatch(DummyPacket())
        fsm.dispatch(DummyPacket())

        self.assertEquals(1, start_state.check)

        self.assertEqual(trim(
                            """
                            State1 A
                            State2
                            State1 B
                            """
                        ),
                         logger.to_string()
        )

    def test_multiple_substate(self):
        class State2(statemachine.State):
            def __init__(self):
                super(State2, self).__init__()
                self.result = 0

            def on_keep_alive(self):
                self.log("State2")
                yield None

        class State1(statemachine.State):
            def __init__(self):
                super(State1, self).__init__()

            def on_keep_alive(self):
                self.log("State1 A")
                state = yield State2()
                self.log("State1 B")
                state = yield State2()
                self.log("State1 C")


        logger = FsmLogger()

        start_state = State1()
        fsm = statemachine.StateMachine(start_state)

        fsm.logger = logger

        fsm.dispatch(DummyPacket())
        fsm.dispatch(DummyPacket())
        fsm.dispatch(DummyPacket())

        self.assertEqual(trim(
            """
            State1 A
            State2
            State1 B
            State2
            State1 C
            """
        ),
                         logger.to_string()
        )
