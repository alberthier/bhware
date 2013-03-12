# encoding: utf-8

import logger
import os
import imp
import inspect
import datetime

#TODO : improve import mechanism by using this tutorial : http://www.doughellmann.com/PyMOTW/imp/
# then when could remove sys.path manipulations

def instantiate_state_machine(state_machine_name, eventloop):
    state_machines_dir = os.path.join(os.path.dirname(__file__), "statemachines")
    state_machine_file = os.path.join(state_machines_dir, state_machine_name + ".py")
    state_machine_module = imp.load_source(state_machine_name, state_machine_file)
    for (item_name, item_type) in inspect.getmembers(state_machine_module):
        if inspect.isclass(item_type) and issubclass(item_type, State) and item_name == "Main":
            root_state = item_type()
            root_state.event_loop = eventloop
            logger.log("Successfully instatiated state '{}' from file '{}'".format(item_name, state_machine_file))
            return root_state
    else:
        logger.log("No 'Main' state found in '{}'".format(state_machine_file))
    return None


class StateGenerator(object):
    def __init__(self, fsm):
        """

        :param fsm: StateMachine
        """
        self.fsm = fsm
        self.started = False
        self.generator = None

    def start(self):
        logger.log('Starting StateGenerator')
        if not self.started :
            self.fsm.push_state(self.fsm.root_state)
            self.generator = iter(self)
            next(self.generator)

    def __iter__(self):
        previous_value = None

        while True :
            packet = (yield)

            generator = packet.dispatch(self.fsm.current_state)

            self.fsm.current_state.current_method = generator

            while True :
                try :
                    if not generator :
                        logger.log('We are not in a generator, staying in same state, '
                                   'waiting for new packet')
                        break

                    new_state = generator.send(previous_value)

                    # yield None veut dire -> Exit State
                    if new_state :
                        previous_value = None
                        self.fsm.push_state(new_state)
                        break
                    else :
                        previous_value = self.fsm.current_state
                        self.fsm.pop_state()
                        generator = self.fsm.current_state.current_method
                except StopIteration :
                    logger.log('Generator has stopped, staying in same state, waiting for new packet')
                    break



    def as_generator(self):
        self.start()
        return self.generator



class StateMachine(object):
    def __init__(self, root_state):
        self.state_history = []
        """:type: self.state_history : list of State"""
        self.root_state = root_state
        self.return_value = None
        self.logger = None
        self.state_generator = StateGenerator(self).as_generator()


    def init_state(self, s):
        s.fsm = self

    def log(self, s):
        if self.logger :
            self.logger.log(s)

    @property
    def current_state(self):
        return self.state_history[-1] if self.state_history else None

    def dispatch(self, packet):
        self.state_generator.send(packet)

    def push_state(self, state):
        logger.log("Switching to state {}".format(state.name))
        self.init_state(state)
        self.state_history.append(state)
        state.on_enter()

    def pop_state(self):
        logger.log("Exiting state {}".format(self.current_state.name))
        self.state_history.pop()




class State(object):

    def __init__(self):
        self.event_loop = None
        self.sub_state = None
        self.parent_state = None
        self.short_description = None
        self.current_method = None
        self.fsm = None

    @property
    def name(self):
        return self.__class__.__name__

    def log(self, s):
        """
        Log to state machine
        :param s: state machine
        """
        self.fsm.log(s)

    def get_short_description(self):
        if not self.short_description :
            return self.__doc__
        return self.short_description


    def switch_to_state(self, new_state):
        self.on_exit()
        new_state.event_loop = self.event_loop
        new_state.sub_state = None
        new_state.parent_state = self.parent_state
        self.parent_state.sub_state = new_state
        logger.log("Switching to state {}".format(type(new_state).__name__))
        self.event_loop.state_history.append(new_state)
        new_state.on_enter()


    def switch_to_substate(self, new_state):
        new_state.event_loop = self.event_loop
        new_state.sub_state = None
        new_state.parent_state = self
        self.sub_state = new_state
        logger.log("Pushing sub-state {}".format(type(new_state).__name__))
        self.event_loop.state_history.append(new_state)
        new_state.on_enter()


    def exit_substate(self, exit_status = None):
        logger.log("Poping sub-state {}".format(type(self).__name__))
        if exit_status is not None :
            logger.log("Substate exit status = {}".format(exit_status))
        self.parent_state.sub_state = None
        if not self.event_loop.state_history[:-1] is self :
            self.event_loop.state_history.append(self)
        self.parent_state.on_exit_substate(self)


    def send_packet(self, packet):
        self.event_loop.send_packet(packet)


    def robot(self):
        return self.event_loop.robot


    def on_enter(self):
        pass


    def on_exit(self):
        pass


    def on_exit_substate(self, substate):
        pass


    def on_timer_tick(self):
        pass


    def on_opponent_in_front(self, packet):
        pass


    def on_opponent_in_back(self, packet):
        pass


    #noinspection PyUnusedLocal
    def on_opponent_disapeared(self, opponent, is_in_front):
        pass
