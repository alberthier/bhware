# encoding: utf-8

import logger
import os
import imp
import inspect
import datetime


#TODO : improve import mechanism by using this tutorial : http://www.doughellmann.com/PyMOTW/imp/
# then when could remove sys.path manipulations


def instantiate_state_machine(state_machine_name):
    state_machines_dir = os.path.join(os.path.dirname(__file__), "statemachines")
    state_machine_file = os.path.join(state_machines_dir, state_machine_name + ".py")
    state_machine_module = imp.load_source(state_machine_name, state_machine_file)
    main_state = None
    end_of_match_state = None
    for (item_name, item_type) in inspect.getmembers(state_machine_module):
        if inspect.isclass(item_type) and issubclass(item_type, State):
            if item_name == "Main":
                main_state = item_type()
                logger.log("Successfully instatiated state '{}' from file '{}'".format(item_name, state_machine_file))
            elif item_name == "EndOfMatch":
                end_of_match_state = item_type()
                logger.log("Successfully instatiated state '{}' from file '{}'".format(item_name, state_machine_file))
            if main_state != None and end_of_match_state != None:
                break
    if main_state == None:
        logger.log("Error: no 'Main' state found in '{}'".format(state_machine_file))
    if end_of_match_state == None:
        logger.log("Error: no 'EndOfMatch' state found in '{}'".format(state_machine_file))
    return (main_state, end_of_match_state)


class Stay(object):
    pass



class StateGenerator(object):
    def __init__(self, fsm):
        """

        :param fsm: StateMachine
        """
        self.fsm = fsm
        self.started = False
        self.generator = None

    def log(self,packet,s):
        if not "KeepAlive" in packet.__class__.__name__ :
            logger.log("[{}] {}".format(self.fsm.current_state.name,s))

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

            self.log(packet, "Got packet {}".format( str(packet)))
            generator = packet.dispatch(self.fsm.current_state)
            self.log(packet, "Dispatch return value = {}".format(str(generator)))

            self.fsm.current_state.current_method = generator

            while True :
                try :
                    if not generator :
                        self.log(packet, 'No current generator, staying in same state, waiting for new packet')
                        break

                    new_state = generator.send(previous_value)

                    # yield None veut dire -> Exit State
                    if new_state :
                        self.log(packet, 'Got new state to enter : {}'.format(new_state.name))
                        previous_value = None
                        # on_enter can yield a generator
                        generator = self.fsm.push_state(new_state)
                        if generator :
                            next(generator)
                        self.fsm.current_state.current_method = generator
                        self.log(packet, "on_enter return value = {}".format(generator))
                        # TODO  : gérer yield dans on_enter (récursivité ?)
                    else :
                        self.log(packet, 'Yield None --> pop')
                        previous_value = self.fsm.current_state
                        self.fsm.pop_state()
                        generator = self.fsm.current_state.current_method
                        self.log(packet, "Generator is {}".format(generator))
                        if generator :
                            next(generator)
                except StopIteration :
                    self.log(packet, 'Generator has stopped, staying in same state, waiting for new packet')
                    break



    def as_generator(self):
        self.start()
        return self.generator



class StateMachine(object):
    def __init__(self, event_loop, root_state):
        self.state_stack = []
        """:type: self.state_stack : list of State"""
        self.state_history = []
        """:type: self.state_history : list of State"""
        self.event_loop = event_loop
        self.root_state = root_state
        self.return_value = None
        self.logger = None
        self.state_generator = StateGenerator(self).as_generator()


    def init_state(self, s):
        s.fsm = self
        s.event_loop = self.event_loop

    def log(self, s):
        if self.logger :
            self.logger.log(s)

    @property
    def current_state(self):
        """
        :return: Current state
        :rtype: State
        """
        return self.state_stack[-1] if self.state_stack else None


    def on_timer_tick(self):
        if self.current_state is not None:
            generator = self.current_state.on_timer_tick()
            self.process(generator)


    def on_opponent_detected(self, packet, opponent_direction, x, y):
        if self.current_state is not None:
            generator = self.current_state.on_opponent_in_front(packet)
            self.process(generator)


    def on_opponent_disapeared(self, opponent, opponent_direction):
        if self.current_state is not None:
            generator = self.current_state.on_opponent_in_back(packet)
            self.process(generator)


    def on_opponent_disapeared(self, opponent, is_in_front):
        if self.current_state is not None:
            generator = self.current_state.on_opponent_disapeared(opponent, is_in_front)
            self.process(generator)


    def on_packet(self, packet):
        if self.current_state is not None:
            generator = packet.dispatch(self.current_state)
            self.process(generator)
            # self.state_generator.send(packet)


    def push_state(self, state):
        logger.log("Switching to state {new} ({old} -> {new})".format(new=state.name,
                                                                      old=self.current_state.name if self.current_state
                                                                            else "" )
        )
        self.init_state(state)
        self.state_stack.append(state)
        self.state_history.append(state)
        return state.on_enter()

    def pop_state(self):
        previous_state = self.current_state
        previous_state.on_exit()
        self.state_stack.pop()
        new_state = self.current_state
        logger.log("Exiting state {previous} ({previous} -> {current})".format(previous=previous_state.name,
                                                                               current=new_state.name))
        
    def log(self,s):
        # if not "KeepAlive" in packet.__class__.__name__ :
        #     logger.log("[{}] {}".format(self.fsm.current_state.name,s))
        logger.log("[{}] {}".format(self.current_state.name,s))

    def process(self, generator):
        previous_value = None
    
        while generator:

            try:
                new_state = generator.send(previous_value)
    
                # yield None veut dire -> Exit State
                if new_state:
                    self.log( 'Got new state to enter : {}'.format(new_state.name))
                    previous_value = None
                    # on_enter can yield a generator
                    self.current_state.current_method = generator
                    generator = self.push_state(new_state)
                    # if generator:
                    #     next(generator)
                    # self.current_state.current_method = generator
                    self.log( "on_enter return value = {}".format(generator))
                else:
                    self.log( 'Yield None --> pop')
                    previous_value = self.current_state
                    self.pop_state()
                    generator = self.current_state.current_method
                    self.log( "Generator is {}".format(generator))
                    # if generator:
                    #     next(generator)
            except StopIteration:
                self.log( 'Generator has stopped, staying in same state, waiting for new packet')
                break

        
        
class FakeEventLoop(object):
    def send_packet(self, packet):
        raise Exception("Ooops - don't send a packet from here !")

fsm = None
event_loop = FakeEventLoop()


class State(object):

    def __init__(self):
        self.fsm = fsm
        self.event_loop = event_loop
        self.short_description = None
        self.current_method = None

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


    # def switch_to_state(self, new_state):
    #     self.on_exit()
    #     new_state.event_loop = self.event_loop
    #     new_state.sub_state = None
    #     new_state.parent_state = self.parent_state
    #     self.parent_state.sub_state = new_state
    #     logger.log("Switching to state {}".format(type(new_state).__name__))
    #     # self.event_loop.state_history.append(new_state)
    #     new_state.on_enter()
    #
    #
    # def switch_to_substate(self, new_state):
    #     new_state.event_loop = self.event_loop
    #     new_state.sub_state = None
    #     new_state.parent_state = self
    #     self.sub_state = new_state
    #     logger.log("Pushing sub-state {}".format(type(new_state).__name__))
    #     # self.event_loop.state_history.append(new_state)
    #     new_state.on_enter()
    #
    #
    # def exit_substate(self, exit_status = None):
    #     logger.log("Poping sub-state {}".format(type(self).__name__))
    #     if exit_status is not None :
    #         logger.log("Substate exit status = {}".format(exit_status))
    #     self.parent_state.sub_state = None
    #     # if not self.event_loop.state_history[:-1] is self :
    #     #     self.event_loop.state_history.append(self)
    #     self.parent_state.on_exit_substate(self)


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


    def on_opponent_detected(self, packet, opponent_direction, x, y):
        pass


    def on_opponent_disapeared(self, opponent, opponent_direction):
        pass


    #noinspection PyUnusedLocal
    def on_opponent_disapeared(self, opponent, is_in_front):
        pass
