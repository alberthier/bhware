# encoding: utf-8

import math

import statemachine
import packets
import position
import logger
import commonstates
import goalmanager

from definitions import *
from commonstates import *
from position import *

X_OPEN_GIFTS_START = 1.83
Y_OPEN_GIFTS_START = 2.3

X_START = BLUE_START_X
Y_START = BLUE_START_Y


class Main(statemachine.State):

    def on_enter(self):
        statemachine.StateMachine(self.event_loop, "barman", side = SIDE_RIGHT)

        gm  = goalmanager.GoalManager(self.fsm.event_loop)
        self.robot.goal_manager = gm

        # we may also handle each gift on its own

        # gm.harvesting_goals.append(goalmanager.Goal("KICK_GIFTS", 1.0, X_OPEN_GIFTS_START, Y_OPEN_GIFTS_START,
        #                                             DIRECTION_BACKWARDS,OpenGifts))

        # gm.harvesting_goals.append(goalmanager.Goal("KICK_GIFTS", 1.0, X_START, 0.5,
        #                                     DIRECTION_BACKWARDS,OpenGiftsLeft, navigate = False))



    def on_device_ready(self, packet):
        # yield CalibratePosition(X_START)
        yield DefinePosition(X_START, 0.05, math.pi/2)
        yield AntiBlocking(True)


    def on_start(self, packet):
        gm = self.robot.goal_manager
        # yield SpeedControl(0.5)
        yield Timer(2000) # 10s
        yield TakeGlasses()
        # yield SpeedControl()
        # yield LookAtOpposite(X_OPEN_GIFTS_START, Y_OPEN_GIFTS_START)
        yield Deposit()
        yield FindNextGoal()

        # yield EndOfMatch()




class Deposit(statemachine.State):

    def on_enter(self):
        yield AntiBlocking(False)
        yield LookAt(BLUE_START_X, BLUE_START_Y)
        yield AntiBlocking(True)
        while True :
            move = yield MoveLineTo(BLUE_START_X, BLUE_START_Y + 0.15)
            if move.exit_reason == TRAJECTORY_DESTINATION_REACHED :
                break
        # yield MoveLineTo(1.66, 0.38)
        yield DepositGlasses()
        yield MoveRelative(-0.3, DIRECTION_BACKWARDS)
        yield None




class TakeGlasses(statemachine.State):

    def on_enter(self):
        # yield MoveLineTo(BLUE_START_X, Y_OPEN_GIFTS_START)
        yield MoveLineTo(BLUE_START_X, 1.0)
        yield Timer(1000.0)
        yield MoveLineTo(BLUE_START_X, 1.93)
        yield Timer(1000.0)
        yield None



class GotoGiftOpenStart(statemachine.State):

    def on_enter(self):
        yield MoveLineTo(X_OPEN_GIFTS_START, Y_OPEN_GIFTS_START)
        yield None

class OpenGifts(statemachine.State):

    X_VALUE = X_OPEN_GIFTS_START
    Y_VALUES = [Y_OPEN_GIFTS_START, 1.79, 1.2, 0.62]

    def __init__(self, goal):
        self.goal = goal
        self.exit_reason = GOAL_FAILED

    def on_enter(self):
        gm = self.robot.goal_manager

        self.gift_opener_side = GIFT_OPENER_POSITION_RIGHT \
            if self.robot.team == TEAM_BLUE else GIFT_OPENER_POSITION_LEFT

        yield Rotate(0.0)
        yield GotoGiftOpenStart()
        yield Rotate(math.pi/2)
        # yield GiftOpener(self.gift_opener_side)
        # yield GiftOpener(GIFT_OPENER_POSITION_IDLE)

        points = [ (self.X_VALUE, y) for y in self.Y_VALUES ]
        move = MoveLine(points, direction=DIRECTION_BACKWARDS)
        move.on_waypoint_reached = self.on_waypoint_reached
        move.on_gift_opener = self.on_gift_opener

        yield move

        if move.exit_reason == TRAJECTORY_DESTINATION_REACHED:
            self.exit_reason = GOAL_DONE

        yield GiftOpener(self.gift_opener_side)
        yield GiftOpener(GIFT_OPENER_POSITION_IDLE)
        yield None


    def on_waypoint_reached(self, packet):
        self.send_packet(packets.GiftOpener(position=self.gift_opener_side))


    def on_gift_opener(self, packet):
        if packet.position != GIFT_OPENER_POSITION_IDLE:
            self.send_packet(packets.GiftOpener(position=GIFT_OPENER_POSITION_IDLE))

class OpenGiftsLeft(statemachine.State):

    X_VALUE = X_OPEN_GIFTS_START
    Y_VALUES = [Y_OPEN_GIFTS_START, 1.7, 1.1, 0.5]

    def __init__(self, goal):
        self.goal = goal
        self.exit_reason = GOAL_FAILED

    def on_enter(self):
        gm = self.robot.goal_manager

        self.gift_opener_side = GIFT_OPENER_POSITION_RIGHT \
            if self.robot.team == TEAM_BLUE else GIFT_OPENER_POSITION_LEFT

        yield AntiBlocking(False)
        yield Rotate(math.pi)
        yield AntiBlocking(True)
        yield SpeedControl(0.2)
        yield MoveLineTo(2.0, self.robot.pose.virt.y, DIRECTION_BACKWARDS)
        yield DefinePosition( 2.0 - ROBOT_CENTER_X, None, math.pi)

        yield SpeedControl()

        yield MoveLineTo(self.X_VALUE, self.Y_VALUES[-1], DIRECTION_FORWARDS)

        yield AntiBlocking(False)
        yield Rotate(math.pi/2)
        yield AntiBlocking(True)

        points = reversed([ (self.X_VALUE, y ) for y in self.Y_VALUES ])
        move = MoveLine(points, direction=DIRECTION_FORWARDS)
        move.on_waypoint_reached = self.on_waypoint_reached
        move.on_gift_opener = self.on_gift_opener

        yield move

        if move.exit_reason == TRAJECTORY_DESTINATION_REACHED:
            self.exit_reason = GOAL_DONE

        yield GiftOpener(self.gift_opener_side)
        yield GiftOpener(GIFT_OPENER_POSITION_IDLE)
        yield None


    def on_waypoint_reached(self, packet):
        self.send_packet(packets.GiftOpener(position=self.gift_opener_side))


    def on_gift_opener(self, packet):
        if packet.position != GIFT_OPENER_POSITION_IDLE:
            self.send_packet(packets.GiftOpener(position=GIFT_OPENER_POSITION_IDLE))


class EndOfMatch(statemachine.State):

    def on_enter(self):
        yield StopAll()

