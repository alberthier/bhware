#!/usr/bin/env python
# encoding: utf-8

import logger
import os
import imp
import inspect

def instantiate_state_machine(state_machine_name, eventloop):
    state_machines_dir = os.path.join(os.path.dirname(__file__), "statemachines")
    state_machine_file = os.path.join(state_machines_dir, state_machine_name + ".py")
    state_machine_module = imp.load_source(state_machine_name, state_machine_file)
    for (item_name, item_type) in inspect.getmembers(state_machine_module):
        if inspect.isclass(item_type) and issubclass(item_type, StateMachine):
            fsm = item_type()
            fsm.event_loop = eventloop
            logger.log("Successfully instatiated state machine '{0}' from file '{1}'".format(item_name, state_machine_file))
            return fsm


class StateMachine(object):

    def __init__(self, start_state_ctor, *args):
        self.state = start_state_ctor(*args)
        self.state.fsm = self
        self.event_loop = None


    def start(self):
        logger.log("Starting state machine '{0}'".format(self.__class__.__name__))
        self.state.on_enter()




class State(object):

    def __init__(self):
        self.fsm = None


    def switch_to_state(self, new_state_ctor, *args):
        self.fsm.state.on_exit()
        self.fsm.state = new_state_ctor(*args)
        logger.log("Switching to state {0}".format(self.fsm.state.__class__.__name__) )
        self.fsm.state.fsm = self.fsm
        self.fsm.state.on_enter()


    def send_packet(self, packet):
        self.fsm.event_loop.send_packet(packet)


    def robot(self):
        return self.fsm.event_loop.robot


    def on_enter(self):
        pass


    def on_exit(self):
        pass


    def on_device_busy(self):
        pass


    def on_device_ready(self, team):
        pass


    def on_start(self, team):
        pass


    def on_goto_started(self):
        pass


    def on_goto_finished(self, reason, pose):
        pass


    def on_blocked(self, side):
        pass


    def on_keep_alive(self, current_pose, match_started, match_time):
        pass


    def on_piece_detected(self, start_pose, start_distance, end_pose, end_distance, sensor, angle):
        pass


    def on_piece_stored(self, piece_count):
        pass


    def on_turret_detect(self, angle):
        pass
