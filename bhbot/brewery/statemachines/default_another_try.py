#!/usr/bin/env python
# encoding: utf-8

import math

import statemachine
import packets
import trajectory
import logger

from collections import deque

from definitions import *

class DefaultStateMachine(statemachine.StateMachine):
    """Default state machine"""
    def __init__(self):
        statemachine.StateMachine.__init__(self, WaitDeviceReady)

class WaitDeviceReady(statemachine.State):

    def on_device_ready(self, team):
        self.switch_to_state(WaitStart)


class WaitStart(statemachine.State):

    def on_start(self, team):
        self.switch_to_state(WaitFirstKeepAlive)

class WaitFirstKeepAlive(statemachine.State):

    def on_keep_alive(self, current_pose, match_started, match_time):
        self.switch_to_machine(HomologationMode)
        # self.switch_to_state(HomologationStart)


class HomologationMode(statemachine.StateMachine):
    """Mode homologation"""
    def __init__(self):
        statemachine.StateMachine.__init__(self, HomologationStart)

class HomologationStart(statemachine.State) :
    """Start of homologation"""
    def __init__(self):
        statemachine.State.__init__(self)
    def on_enter(self):
        self.switch_to_machine(TrajectoryWalker(HomologationEnd, homologation_trajectory))

    def on_submachine_exit(self, machine):
        if(len(machine.points) == 0):
            logging.log("We arrived to destination")
        else :
            logging.log("We didn't arrive to destination")
        self.switc          

# class HomologationEnd(statemachine.State):
#     def on_enter(self,???) :

class TrajectoryWalker(statemachine.StateMachine):
    def __init__(self, points):
        statemachine.State.__init__(self, TrajectoryWalk)
        self.points = deque(points)
        

class TrajectoryWalk(statemachine.State):
    """Walk a path"""
    def __init__(self):
        statemachine.State.__init__(self)

    def on_enter(self):
        self.robot.goto(self.fsm.points[Ø])

    def on_goto_finished(self):
        self.fsm.points.popleft()
        if len(self.fsm.points) > 0 :
            self.robot.goto(self.fsm.points[Ø])
        else :
            self.fsm.exit()

    def on_evit_detected(self):
        #exit, remaining points are dropped
        self.fsm.exit()

    # def on_turret_detect(self, detect_angle):
    #     # self.wait(time=5
    #     #          ,success=self.continue_path
    #     #          ,failure=lambda: self.fsm.exit()
    #     #          ,condition=lambda : angle.is_aligned(vector(self.robot.pose,detect_angle), self.robot.current_direction, constants.90_DEGREES))
    #     #          )
    #     if self.robot.is_in_my_way(detect_angle) :
    #         self.switch_to_state(WaitOpponentMove)

    def on_opponent_moved(self, position):
        if self.robot.is_in_my_way(position) :
            self.switch_to_state(WaitOpponentMove)
    
    # def continue_path(self):
    #     self.robot.goto(self.robot.current_destination)

class WaitOpponentMove(statemachine.State):
    def on_enter(self):
        self.set_timer("opponent_move",5)
    
    def on_timeout(self,name):
        logging.log("Opponent didn't move enough")
        self.fsm.exit()

    def on_opponent_moved(self,position):
        if not self.robot.is_in_my_way(position) :
            self.switch_to_state(TrajectoryWalk)




class ConstructMode(statemachine.StateMachine):
    """Mode construction"""
    def __init__(self):
        statemachine.StateMachine.__init__(self, ConstructIdle)

class RedemptionMode(statemachine.StateMachine):
    """Mode Rédempteur"""
    def __init__(self):
        statemachine.StateMachine.__init__(self, RedemptorIdle)


class EndOfTheWorldMode(statemachine.StateMachine):
    """Mode End of the World"""
    def __init__(self):
        statemachine.StateMachine.__init__(self, EndOfTheWorldIdle)

class DropWhenOutOfTime(statemachine.StateMachine):
    """ Super-state which triggers when out of time"""
    def __init__(self):
        statemachine.StateMachine.__init__(self,XXX)

class EndOfTheWorldIdle(statemachine.State):
    """Searching for a place to land"""
    def on_enter(self):
        squares = geo.get_nearest_bonus_squares(self.robot)
        self.robot.destination_square = squares[0]
        self.switch_to_state()


# class GotoFieldMove(statemachine.State):
#     def on_enter(self):
#         self.robot().forward(0.4)

#     def on_goto_finished(self, reason, pose):
#         self.switch_to_state(GotoFieldRotate)


# class GotoFieldRotate(statemachine.State):

#     def on_enter(self):
#         angle = math.pi / 6.0
#         if self.robot().team == TEAM_RED:
#             angle = -angle
#         self.robot().rotate(angle)


#     def on_goto_finished(self, reason, pose):
#         self.switch_to_state(GotoFieldMove2)



# class GotoFieldMove2(statemachine.State):

#     def on_enter(self):
#         self.robot().forward(0.4)


#     def on_goto_finished(self, reason, pose):
#         self.switch_to_state(GotoFieldRotate2)



# class GotoFieldRotate2(statemachine.State):

#     def on_enter(self):
#         angle = math.pi / 3.0
#         if self.robot().team == TEAM_RED:
#             angle = -angle
#         self.robot().rotate(angle)


#     def on_goto_finished(self, reason, pose):
#         self.switch_to_state(Moving)



# class Rotate(statemachine.State):

#     def on_enter(self):
#         angle = math.pi / 2.0
#         if self.robot().team == TEAM_RED:
#             angle = -angle
#         self.robot().rotate(angle)


#     def on_goto_finished(self, reason, pose):
#         self.switch_to_state(Moving)


# class Moving(statemachine.State):

#     def on_enter(self):
#         self.robot().forward(0.3)


#     def on_goto_finished(self, reason, pose):
#         self.switch_to_state(Rotate)




