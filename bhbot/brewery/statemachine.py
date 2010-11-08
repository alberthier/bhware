#!/usr/bin/env python
# encoding: utf-8




class StateMachine(object):

    def __init__(self, start_state_ctor, owner_state = None):
        self.current_state = start_state_ctor()
        self.current_state.on_enter()
        if owner_state != None:
            self.parent_fsm = owner_state.fsm
        else:
            self.parent_fsm = None




class State(object):

    def __init__(self, fsm):
        self.fsm = fsm
        self.sub_fsm = None


    def switch_to_state(self, new_state_ctor):
        self.fsm.current_state.on_exit()
        self.fsm.current_state = new_state_ctor()
        self.fsm.current_state.on_enter()


    def exit_fsm(self):
        self.fsm.current_state.on_exit()
        if self.fsm.parent_fsm != None:
            my_fsm = self.fsm.parent_fsm.current_state.sub_fsm
            self.fsm.parent_fsm.on_sub_fsm_exited()
            if self.fsm.parent_fsm.current_state.sub_fsm == my_fsm:
                self.fsm.parent_fsm.current_state.sub_fsm = None


    def on_enter(self):
        pass


    def on_exit(self):
        pass


    def on_sub_fsm_exited(self):
        pass


    def on_device_busy(self):
        if self.sub_fsm != None:
            self.sub_fsm.current_state.on_device_busy()


    def on_device_ready(self, team):
        if self.sub_fsm != None:
            self.sub_fsm.current_state.on_device_ready(team)


    def on_start(self, team):
        if self.sub_fsm != None:
            self.sub_fsm.current_state.on_start(team)


    def on_goto_started(self):
        if self.sub_fsm != None:
            self.sub_fsm.current_state.on_goto_started()


    def on_goto_finished(self, reason):
        if self.sub_fsm != None:
            self.sub_fsm.current_state.on_goto_finished(reason)


    def on_blocked(self, side):
        if self.sub_fsm != None:
            self.sub_fsm.current_state.on_blocked(side)


    def on_keep_alive(self, match_started, match_time):
        if self.sub_fsm != None:
            self.sub_fsm.current_state.on_keep_alive(match_started, match_time)


    def on_turret_detect(self, mean_angle, angular_size):
        if self.sub_fsm != None:
            self.sub_fsm.current_state.on_turret_detect(mean_angle, angular_size)

