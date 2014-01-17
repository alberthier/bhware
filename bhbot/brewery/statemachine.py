# encoding: utf-8

import logger
import os
import imp
import inspect
import datetime




class StateMachine(object):

    def __init__(self, event_loop, name, **kwargs):
        self.state_stack = []
        """:type: self.state_stack : list of State"""
        self.state_history = []
        """:type: self.state_history : list of State"""
        self.event_loop = event_loop
        self.name = name
        for k, v in kwargs.items():
            setattr(self, k, v)
        (main_state, self.end_of_match_state) = self.instantiate_state_machine(self.name)
        if main_state is not None:
            self.event_loop.fsms.append(self)
            self.process(self.push_state(main_state))


    def instantiate_state_machine(self, state_machine_name):
        state_machines_dir = os.path.join(os.path.dirname(__file__), "statemachines")
        state_machine_file = os.path.join(state_machines_dir, state_machine_name + ".py")
        state_machine_module = imp.load_source(state_machine_name, state_machine_file)
        main_state = None
        end_of_match_state = None
        for (item_name, item_type) in inspect.getmembers(state_machine_module):
            if inspect.isclass(item_type) and issubclass(item_type, State):
                if item_name == "Main":
                    main_state = item_type()
                    self.log("Successfully instatiated state '{}' from file '{}'".format(item_name, state_machine_file))
                elif item_name == "EndOfMatch":
                    end_of_match_state = item_type()
                    self.log("Successfully instatiated state '{}' from file '{}'".format(item_name, state_machine_file))
                if main_state != None and end_of_match_state != None:
                    break
        if main_state is None:
            self.log("Error: no 'Main' state found in '{}'".format(state_machine_file))
        if end_of_match_state is None:
            self.log("Warning: no 'EndOfMatch' state found in '{}'".format(state_machine_file))
        return (main_state, end_of_match_state)


    def switch_to_end_of_match(self):
        self.state_stack = []
        if self.end_of_match_state is not None:
            self.process(self.push_state(self.end_of_match_state))


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


    def log(self, msg):
        logger.log(self.name + ": " + msg)


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
        self.log("Switching to state {new} ({old} -> {new})".format(new = state.name,
                                                                      old = self.current_state.name if self.current_state else "" ))
        self.init_state(state)
        self.state_stack.append(state)
        self.state_history.append(state.name)
        return state.on_enter()


    def pop_state(self):
        previous_state = self.current_state
        previous_state.on_exit()
        self.state_stack.pop()
        new_state = self.current_state
        self.log("Exiting state {previous} ({previous} -> {current})".format(previous = previous_state.name,
                                                                               current = new_state.name))

    def process(self, generator):
        previous_value = None
        while generator:
            try:
                new_state = generator.send(previous_value)
                if isinstance(new_state, State):
                    previous_value = None
                    # on_enter can yield a generator
                    self.current_state.fsm_current_method = generator
                    generator = self.push_state(new_state)
                elif new_state is None:
                    # yield None means exit current State
                    previous_value = self.current_state
                    self.pop_state()
                    generator = self.current_state.fsm_current_method
            except StopIteration:
                generator = None




class State(object):

    @property
    def name(self):
        return self.__class__.__name__


    def log(self, msg):
        self.fsm.log(msg)


    def dbg(self, msg):
        self.fsm.dbg(msg)


    def send_packet(self, packet):
        self.event_loop.send_packet(packet)


    def on_enter(self):
        pass


    def on_exit(self):
        pass


    def on_exit_substate(self, substate):
        pass


    def on_timer_tick(self):
        pass
