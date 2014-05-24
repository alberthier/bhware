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
from tools import *

import statemachines.commontests
import statemachines.martytests



BORDER_FIRE_DIST = 0.15



class Main(statemachine.State):

    def on_enter(self):
        statemachine.StateMachine(self.event_loop, "interbot")
        statemachine.StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_MAIN)
        statemachine.StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_SECONDARY)
        statemachine.StateMachine(self.event_loop, "relaytoggler")


    def on_device_ready(self, packet):
        gm = self.robot.goal_manager

        #                      |       ID       |Weight|                            X                    |                           Y                     |     Direction    |     State     | Ctor parameters  |Shared|Navigate|
        gm.add(goalmanager.Goal("BorderFireW"   ,     1,                                              0.8,                ROBOT_CENTER_X + BORDER_FIRE_DIST, DIRECTION_FORWARD, PullBorderFire, (-math.pi / 2.0,), False,    True))
        gm.add(goalmanager.Goal("BorderFireSW"  ,    10, FIELD_X_SIZE - ROBOT_CENTER_X - BORDER_FIRE_DIST,                                              1.3, DIRECTION_FORWARD, PushBorderFire,            (0.0,), False,    True))
        gm.add(goalmanager.Goal("BorderFireSE"  ,    10, FIELD_X_SIZE - ROBOT_CENTER_X - BORDER_FIRE_DIST,                                       sym_y(1.3), DIRECTION_FORWARD, PullBorderFire,            (0.0,), False,    True))
        gm.add(goalmanager.Goal("BorderFireE"   ,    10,                                              0.8, FIELD_Y_SIZE - ROBOT_CENTER_X - BORDER_FIRE_DIST, DIRECTION_FORWARD, PushBorderFire,  (math.pi / 2.0,), False,    True))
        gm.add(goalmanager.Goal("FieldFireW"    ,    10,                      1.1 - ROBOT_CENTER_X - 0.03,                                              0.4, DIRECTION_FORWARD, PushFieldFire ,            (0.0,), False,    True))
        gm.add(goalmanager.Goal("FieldFireW"    ,    10,                      1.1 + ROBOT_CENTER_X + 0.03,                                              0.4, DIRECTION_FORWARD, PullFieldFire ,        (math.pi,), False,    True))
        gm.add(goalmanager.Goal("FieldFireSW"   ,    10,                                              1.6,                      0.9 - ROBOT_CENTER_X - 0.03, DIRECTION_FORWARD, PullFieldFire ,  (math.pi / 0.2,), False,    True))
        gm.add(goalmanager.Goal("FieldFireSW"   ,    10,                                              1.6,                      0.9 + ROBOT_CENTER_X + 0.03, DIRECTION_FORWARD, PushFieldFire , (-math.pi / 2.0,), False,    True))
        gm.add(goalmanager.Goal("FieldFireSE"   ,    10,                                              1.6,       FIELD_Y_SIZE - 0.9 - ROBOT_CENTER_X - 0.03, DIRECTION_FORWARD, PullFieldFire ,  (math.pi / 2.0,), False,    True))
        gm.add(goalmanager.Goal("FieldFireSE"   ,    10,                                              1.6,       FIELD_Y_SIZE + 0.9 + ROBOT_CENTER_X + 0.03, DIRECTION_FORWARD, PushFieldFire , (-math.pi / 2.0,), False,    True))
        gm.add(goalmanager.Goal("FieldFireE"    ,    10,                      1.1 - ROBOT_CENTER_X - 0.03,                                       sym_y(0.4), DIRECTION_FORWARD, PullFieldFire ,            (0.0,), False,    True))
        gm.add(goalmanager.Goal("FieldFireE"    ,    10,                      1.1 + ROBOT_CENTER_X + 0.03,                                       sym_y(0.4), DIRECTION_FORWARD, PushFieldFire ,        (math.pi,), False,    True))
        gm.add(goalmanager.Goal("HuntTheMammoth",    15,               0.3 + ROBOT_GYRATION_RADIUS + 0.03,                                             2.25, DIRECTION_FORWARD, HuntTheMammoth,              None, False,    True))
        gm.add(goalmanager.Goal("PaintFresco"   ,    20,                                      RED_START_X,                                             1.30, DIRECTION_FORWARD, PaintFresco,              None, False,    True))

        yield AntiBlocking(True)
        yield Trigger(ARM_CLOSE)
        yield CalibratePosition()


    def on_start(self, packet):
        self.yield_at(90000, EndOfMatch())
        logger.log("Starting ...")
        yield FirstHurryToTheOtherMammoth()
        yield ExecuteGoals()




class CalibratePosition(statemachine.State):

    def on_enter(self):
        if IS_HOST_DEVICE_PC:
            yield DefinePosition(RED_START_X, RED_START_Y, RED_START_ANGLE)
        else:
            estimated_start_y = FIELD_Y_SIZE / 2.0
            yield DefinePosition(0.3 + ROBOT_CENTER_X, estimated_start_y, 0.0)
            yield MoveLineTo(RED_START_X, estimated_start_y)
            if self.robot.team == TEAM_RED:
                yield RotateTo(math.pi / 2.0)
            yield SpeedControl(0.2)
            yield MoveLineTo(RED_START_X, 0.0, DIRECTION_BACKWARDS)
            yield SpeedControl()
            yield DefinePosition(None, ROBOT_CENTER_X, math.pi / 2.0)
        yield None




class FirstHurryToTheOtherMammoth(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.ServoControl(*ARM_OPEN))
        move = MoveLineTo(RED_START_X, 2.25)
        yield move
        self.send_packet(packets.ServoControl(*ARM_CLOSE))
        if move.exit_reason == TRAJECTORY_DESTINATION_REACHED:
            yield HuntTheMammoth()
        self.robot.goal_manager.update_goal_status("HuntTheMammoth", GOAL_DONE)
        yield None




class HuntTheMammoth(statemachine.State):

    def on_enter(self):
        if self.robot.team == TEAM_YELLOW:
            yield RotateTo(-math.pi / 2.0)
        yield Trigger(GUN_FIRE)
        self.exit_reason = GOAL_DONE
        yield None




class PaintFresco(statemachine.State):

    def on_enter(self):
        goal = self.robot.goal_manager.get_current_goal()
        yield RotateTo(0.0)
        yield SpeedControl(0.3)
        yield MoveLineTo(ROBOT_CENTER_X - 0.02, goal.y, DIRECTION_BACKWARDS)
        yield Trigger(PAINT_1_FLIP_FLOP_START, PAINT_1_FLIP_FLOP_START)
        yield SpeedControl()
        yield MoveLineTo(0.4, goal.y)
        yield Trigger(PAINT_1_FLIP_FLOP_STOP, PAINT_1_FLIP_FLOP_STOP)
        self.exit_reason = GOAL_DONE
        yield None




class PushBorderFire(statemachine.State):

    def __init__(self, angle):
        self.angle = angle


    def on_enter(self):
        dist = BORDER_FIRE_DIST - 0.05
        goal = self.robot.goal_manager.get_current_goal()
        fx = goal.x + math.cos(self.angle) * dist
        fy = goal.y + math.sin(self.angle) * dist
        yield LookAt(fx, fy)
        yield Trigger(makeServoMoveCommand(ARM, 60))
        yield MoveLineTo(fx, fy)
        yield Trigger(makeServoMoveCommand(ARM, 40))
        yield MoveLineTo(goal.x, goal.y, DIRECTION_BACKWARDS)
        yield Trigger(ARM_CLOSE)
        self.exit_reason = GOAL_DONE
        yield None




class PullBorderFire(statemachine.State):

    def __init__(self, angle):
        self.angle = angle


    def on_enter(self):
        dist = BORDER_FIRE_DIST - 0.05
        goal = self.robot.goal_manager.get_current_goal()
        fx = goal.x + math.cos(self.angle) * dist
        fy = goal.y + math.sin(self.angle) * dist
        yield LookAt(fx, fy)
        yield Trigger(ARM_CLOSE)
        yield MoveLineTo(fx, fy)
        yield Trigger(makeServoMoveCommand(ARM, 50))
        yield MoveLineTo(goal.x, goal.y, DIRECTION_BACKWARDS)
        yield Trigger(ARM_CLOSE)
        self.exit_reason = GOAL_DONE
        yield None




class PushFieldFire(statemachine.State):

    def __init__(self, angle):
        self.angle = angle


    def on_enter(self):
        yield RotateTo(self.angle)
        yield Trigger(ARM_OPEN)
        yield MoveLineRelative(0.05)
        yield Trigger(ARM_CLOSE)
        self.exit_reason = GOAL_DONE
        yield None




class PullFieldFire(statemachine.State):

    def __init__(self, angle):
        self.angle = angle


    def on_enter(self):
        yield RotateTo(self.angle)
        yield MoveLineRelative(0.05)
        yield Trigger(ARM_OPEN)
        yield MoveLineRelative(0.05, DIRECTION_BACKWARDS)
        yield Trigger(ARM_CLOSE)
        self.exit_reason = GOAL_DONE
        yield None




##################################################
# End of match




class EndOfMatch(statemachine.State):

    def on_enter(self):
        yield StopAll();
