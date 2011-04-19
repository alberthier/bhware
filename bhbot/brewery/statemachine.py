#!/usr/bin/env python
# encoding: utf-8

import logger
import os
import imp
import inspect
import datetime
from collections import deque


def instantiate_state_machine(state_machine_name, eventloop):
    state_machines_dir = os.path.join(os.path.dirname(__file__), "statemachines")
    state_machine_file = os.path.join(state_machines_dir, state_machine_name + ".py")
    state_machine_module = imp.load_source(state_machine_name, state_machine_file)
    for (item_name, item_type) in inspect.getmembers(state_machine_module):
        if inspect.isclass(item_type) and issubclass(item_type, State) and item_name == "Main":
            root_state = item_type()
            root_state.event_loop = eventloop
            logger.log("Successfully instatiated state machine '{0}' from file '{1}'".format(item_name, state_machine_file))
            return root_state




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
        logger.log("Switching to state {0}".format(new_state.__class__.__name__))
        new_state.on_enter()


    def switch_to_substate(self, new_state):
        new_state.event_loop = self.event_loop
        new_state.sub_state = None
        new_state.parent_state = self
        self.sub_state = new_state
        logger.log("Switching to sub-state {0}".format(new_state.__class__.__name__))
        new_state.on_enter()


    def exit_substate(self):
        self.parent_state.on_exit_substate(self)
        self.parent_state.sub_state = None


    def send_packet(self, packet):
        self.event_loop.send_packet(packet)


    def robot(self):
        return self.event_loop.robot


    def on_enter(self):
        pass


    def on_exit(self):
        pass


    def on_resettle(self):
        pass


    def on_exit_substate(self, substate):
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




class Timer(State):

    def __init__(self, miliseconds):
        State.__init__(self)
        self.end_time = datetime.datetime.now() + datetime.timedelta(0, 0, 0, miliseconds)


    def on_keep_alive(self, current_pose, match_started, match_time):
        if datetime.datetime.now() > self.end_time:
            self.exit_substate()




class Sequence(State):

    def __init__(self, *args):
        self.substates = deque(args)


    def add(self, substate):
        self.substates.append(substate)


    def on_enter(self):
        self.process_next_substate()

    def on_exit_substate(self):
        self.process_next_substate()


    def process_next_substate(self):
        if len(self.substates) == 0:
            self.exit_substate()
        else:
            self.switch_to_substate(self.substates.popleft())
