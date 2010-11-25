#!/usr/bin/env python
# encoding: utf-8




class StateMachine(object):

    def __init__(self, start_state_ctor):
        self.state = start_state_ctor()
        self.state.fsm = self
        self.event_loop = None


    def start(self):
        self.state.on_enter()




class State(object):

    def __init__(self):
        self.fsm = None


    def switch_to_state(self, new_state_ctor):
        self.fsm.state.on_exit()
        self.fsm.state = new_state_ctor()
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


    def on_goto_finished(self, reason):
        pass


    def on_blocked(self, side):
        pass


    def on_keep_alive(self, current_pose, match_started, match_time):
        pass


    def on_turret_detect(self, mean_angle, angular_size):
        pass
