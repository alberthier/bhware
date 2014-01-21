# encoding: utf-8

import datetime
import imp
import inspect
import logger
import os
import traceback

import eventloop




class StateMachine(object):

    def __init__(self, event_loop, name, **kwargs):
        self.state_stack = []
        """:type: self.state_stack : list of State"""
        self.pending_states = []
        self.event_loop = event_loop
        self.name = name
        for k, v in kwargs.items():
            setattr(self, k, v)
        main_state = self.instantiate_state_machine(self.name)
        if main_state is not None:
            self.event_loop.fsms.append(self)
            self.process(self.push_state(main_state))


    def instantiate_state_machine(self, state_machine_name):
        state_machines_dir = os.path.join(os.path.dirname(__file__), "statemachines")
        state_machine_file = os.path.join(state_machines_dir, state_machine_name + ".py")
        state_machine_module = imp.load_source(state_machine_name, state_machine_file)
        main_state = None
        for (item_name, item_type) in inspect.getmembers(state_machine_module):
            if inspect.isclass(item_type) and issubclass(item_type, State):
                if item_name == "Main":
                    main_state = item_type()
                    self.log("Successfully instatiated state '{}' from file '{}'".format(item_name, state_machine_file))
                    break
        if main_state is None:
            self.log("Error: no 'Main' state found in '{}'".format(state_machine_file))
        return main_state


    def preempt_with_state(self, state):
        self.state_stack = []
        self.process(self.push_state(state))


    def init_state(self, s):
        s.fsm_current_method = None
        s.fsm = self
        s.event_loop = self.event_loop
        s.robot = self.event_loop.robot


    @property
    def current_state(self):
        """
        :return: Current state
        :rtype: State
        """
        return self.state_stack[-1] if self.state_stack else None


    @property
    def current_state_name(self):
        state = self.current_state
        if state != None:
            return state.name
        else:
            return "(None)"


    def log(self, msg):
        logger.log(self.name + ": " + str(msg))


    def log_exception(self, exc):
        self.log("")
        for l in traceback.format_exception(type(exc), exc, None):
            for ll in l.splitlines():
                logger.log(self.name + ": " + ll, "ARM", True)
        self.log("")


    def dbg(self, msg):
        logger.dbg(self.name + ": " + msg)


    def on_timer_tick(self):
        if self.current_state is not None:
            generator = self.current_state.on_timer_tick()
            self.process(generator)


    def on_packet(self, packet):
        if self.current_state is not None:
            generator = packet.dispatch(self.current_state)
            self.process(generator)


    def push_state(self, state):
        self.log("Switching to state {new} ({old} -> {new})".format(old = self.current_state_name, new = state.name))
        self.init_state(state)
        self.state_stack.append(state)
        return state.on_enter()


    def pop_state(self):
        previous_state = self.current_state
        previous_state.on_exit()
        self.state_stack.pop()
        self.log("Exiting state {old} ({old} -> {new})".format(old = previous_state.name, new = self.current_state_name))
        if self.current_state is not None:
            return (previous_state, self.current_state.fsm_current_method)
        else:
            return (previous_state, None)

    def process(self, generator):
        previous_state = None
        while generator:
            try:
                new_state = generator.send(previous_state)
                if isinstance(new_state, State):
                    previous_state = None
                    # on_enter can yield a generator
                    self.current_state.fsm_current_method = generator
                    generator = self.push_state(new_state)
                elif new_state is None:
                    # yield None means exit current State
                    previous_state, generator = self.pop_state()
            except StopIteration:
                generator = None
            except Exception as e:
                # On any other Exception, we dump the current generator and pop the state.
                # We will try to continue in this degraded state, hoping we dont break anything
                self.log("An exception occured while in state '{}':".format(self.current_state_name))
                self.log_exception(e)
                self.log("Trying to continue after having popped the state")
                previous_state, generator = self.pop_state()
        if self.current_state is None and len(self.pending_states) == 0:
            logger.log("State machine '{}' has no current state or pending states, exiting".format(self.name))
            self.event_loop.fsms.remove(self)




class Delayed:

    def __init__(self, event_loop, timeout_ms, fsm, state):
        eventloop.Timer(event_loop, timeout_ms, self.on_timeout).start()
        self.fsm = fsm
        self.state = state
        self.fsm.pending_states.append(self)


    def on_timeout(self):
        self.fsm.pending_states.remove(self)
        self.fsm.preempt_with_state(self.state)




class State:

    @property
    def name(self):
        return self.__class__.__name__


    def log(self, msg):
        self.fsm.log(msg)


    def log_exception(self, exc):
        self.fsm.log_exception(exc)


    def dbg(self, msg):
        self.fsm.dbg(msg)


    def send_packet(self, packet):
        self.event_loop.send_packet(packet)


    def yield_at(self, timeout_ms, state):
        Delayed(self.event_loop, timeout_ms, self.fsm, state)


    def on_enter(self):
        pass


    def on_exit(self):
        pass


    def on_timer_tick(self):
        pass
