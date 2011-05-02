#!/usr/bin/env python
# encoding: utf-8

import math

import statemachine
import packets
import trajectory
import logger
import world
import commonstates
from definitions import *




class Main(statemachine.State):

    def on_device_ready(self, team):
        if team == TEAM_RED:
            x = 31.0
            angle = 0.0
        elif team == TEAM_BLUE:
            x = 3000.0 - 31.0
            angle = math.pi
        y = 337 / 2.0 + 50.0
        self.switch_to_substate(commonstates.DefinePosition(x, y, angle))


    def on_exit_substate(self, substate):
        self.switch_to_state(WaitStart())




class WaitStart(statemachine.State):

    def on_start(self, team):
        self.switch_to_state(WaitFirstKeepAlive())



class WaitFirstKeepAlive(statemachine.State):

    def on_keep_alive(self, current_pose, match_started, match_time):
        self.switch_to_state(GotoFieldMove())



class TestHomologationWorld(statemachine.State):

    def on_enter(self):
        self.trajectory = world.world.get_trajectory()
        self.current = None
        self.next_move()


    def next_move(self):
        if self.current == None:
            if not len(self.trajectory) == 0:
                self.current = self.trajectory[0]
                self.trajectory = self.trajectory[1:]
                self.robot().look_at(self.current[0] / 1000.0, self.current[1] / 1000.0)
        else:
            self.robot().move_to(self.current[0] / 1000.0, self.current[1] / 1000.0)
            self.current = None


    def on_goto_finished(self, reason, pose):
        self.next_move()



class GotoFieldGenericGoto(statemachine.State):

    def on_enter(self):
        x = 1.0
        if self.robot().team == TEAM_RED:
            y = 1.3
            angle = -math.pi / 2.0
        elif self.robot().team == TEAM_BLUE:
            y = 3.0 - 1.3
            angle = math.pi / 2.0
        self.robot().goto(x, y, angle, DIRECTION_FORWARD)
        self.first = True


    def on_goto_finished(self, reason, pose):
        if self.first:
            self.first = False
            robot = self.robot()
            robot.goto(robot.pose.x, robot.pose.y, robot.pose.angle, DIRECTION_FORWARD)
        else:
            self.switch_to_state(GotoFieldRotate())



class GotoFieldMove(statemachine.State):

    def on_enter(self):
        self.robot().forward(0.4)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(GotoFieldRotate())



class GotoFieldRotate(statemachine.State):

    def on_enter(self):
        angle = math.pi / 6.0
        if self.robot().team == TEAM_RED:
            angle = -angle
        self.robot().rotate(angle)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(GotoFieldMove2())



class GotoFieldMove2(statemachine.State):

    def on_enter(self):
        self.robot().forward(0.4)


    def on_goto_finished(self, reason, pose):
        if reason == REASON_PIECE_FOUND:
            self.switch_to_substate(commonstates.StorePiece1())
        else:
            self.switch_to_state(GotoFieldRotate2())


    def on_exit_substate(self, substate):
        self.switch_to_state(GotoFieldMove3())



class GotoFieldMove3(statemachine.State):

    def on_enter(self):
        self.robot().forward(0.2)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(GotoFieldMove4())



class GotoFieldMove4(statemachine.State):

    def on_enter(self):
        self.robot().backward(0.1)



class GotoFieldRotate2(statemachine.State):

    def on_enter(self):
        angle = math.pi / 3.0
        if self.robot().team == TEAM_RED:
            angle = -angle
        self.robot().rotate(angle)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(GotoFieldMove2())



class Rotate(statemachine.State):

    def on_enter(self):
        angle = math.pi / 2.0
        if self.robot().team == TEAM_RED:
            angle = -angle
        self.robot().rotate(angle)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(RotateQ1())


class RotateQ1(statemachine.State):

    def on_enter(self):
        self.robot().look_at(self.robot().pose.x - 10.0, self.robot().pose.y - 10.0)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(RotateQ2())


class RotateQ2(statemachine.State):

    def on_enter(self):
        self.robot().look_at(self.robot().pose.x + 10.0, self.robot().pose.y - 10.0)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(RotateQ3())


class RotateQ3(statemachine.State):

    def on_enter(self):
        self.robot().look_at(self.robot().pose.x + 10.0, self.robot().pose.y + 10.0)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(RotateQ4())


class RotateQ4(statemachine.State):

    def on_enter(self):
        self.robot().look_at(self.robot().pose.x - 10.0, self.robot().pose.y + 10.0)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(RotateQ1())


class Moving(statemachine.State):

    def on_enter(self):
        self.robot().forward(0.3)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(Rotate())
