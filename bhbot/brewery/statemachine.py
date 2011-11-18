#!/usr/bin/env python
# encoding: utf-8

import logger
import os
import imp
import inspect
import datetime


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




class State(object):

    def __init__(self):
        self.event_loop = None
        self.sub_state = None
        self.parent_state = None


    def switch_to_state(self, new_state):
        self.on_exit()
        new_state.event_loop = self.event_loop
        new_state.sub_state = None
        new_state.parent_state = self.parent_state
        self.parent_state.sub_state = new_state
        logger.log("Switching to state {}".format(type(new_state).__name__))
        new_state.on_enter()


    def switch_to_substate(self, new_state):
        new_state.event_loop = self.event_loop
        new_state.sub_state = None
        new_state.parent_state = self
        self.sub_state = new_state
        logger.log("Pushing sub-state {}".format(type(new_state).__name__))
        new_state.on_enter()


    def exit_substate(self, exit_status = None):
        logger.log("Poping sub-state {}".format(type(self).__name__))
        if exit_status is not None :
            logger.log("Substate exit status = {}".format(exit_status))
        self.parent_state.sub_state = None
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


    def on_opponent_entered(self, angle):
        pass


    def on_opponent_detected(self, angle):
        pass


    def on_opponent_left(self):
        pass
