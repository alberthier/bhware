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




class Main(statemachine.State):

    def on_enter(self):
        statemachine.StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_MAIN)
        statemachine.StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_SECONDARY)
        statemachine.StateMachine(self.event_loop, "relaytoggler")


    def on_device_ready(self, packet):
        gm = self.robot.goal_manager

        #                      |       ID       |Weight|                X                    |                     Y                     |     Direction    |     State     | Ctor parameters  |Shared|Navigate|
        gm.add(goalmanager.Goal("BorderFireW"   ,     1,                                  0.8,                      ROBOT_CENTER_X + 0.03, DIRECTION_FORWARD, PullBorderFire, (-math.pi / 2.0,), False,    True))
        gm.add(goalmanager.Goal("BorderFireSW"  ,     5, FIELD_X_SIZE - ROBOT_CENTER_X - 0.03,                                        1.3, DIRECTION_FORWARD, PushBorderFire,            (0.0,), False,    True))
        gm.add(goalmanager.Goal("BorderFireSE"  ,     5,                                  0.8,       FIELD_Y_SIZE - ROBOT_CENTER_X - 0.03, DIRECTION_FORWARD, PullBorderFire,            (0.0,), False,    True))
        gm.add(goalmanager.Goal("BorderFireE"   ,     5, FIELD_X_SIZE - ROBOT_CENTER_X - 0.03,                         FIELD_Y_SIZE - 1.3, DIRECTION_FORWARD, PushBorderFire,  (math.pi / 2.0,), False,    True))
        gm.add(goalmanager.Goal("FieldFireW"    ,    10,          1.1 - ROBOT_CENTER_X - 0.03,                                        0.4, DIRECTION_FORWARD, PushFieldFire ,            (0.0,), False,    True))
        gm.add(goalmanager.Goal("FieldFireW"    ,    10,          1.1 + ROBOT_CENTER_X + 0.03,                                        0.4, DIRECTION_FORWARD, PullFieldFire ,        (math.pi,), False,    True))
        gm.add(goalmanager.Goal("FieldFireSW"   ,    10,                                  1.6,                0.9 - ROBOT_CENTER_X - 0.03, DIRECTION_FORWARD, PullFieldFire ,  (math.pi / 0.2,), False,    True))
        gm.add(goalmanager.Goal("FieldFireSW"   ,    10,                                  1.6,                0.9 + ROBOT_CENTER_X + 0.03, DIRECTION_FORWARD, PushFieldFire , (-math.pi / 2.0,), False,    True))
        gm.add(goalmanager.Goal("FieldFireE"    ,    10,          1.1 - ROBOT_CENTER_X - 0.03,                         FIELD_Y_SIZE - 0.4, DIRECTION_FORWARD, PullFieldFire ,            (0.0,), False,    True))
        gm.add(goalmanager.Goal("FieldFireE"    ,    10,          1.1 + ROBOT_CENTER_X + 0.03,                         FIELD_Y_SIZE - 0.4, DIRECTION_FORWARD, PushFieldFire ,        (math.pi,), False,    True))
        gm.add(goalmanager.Goal("FieldFireSE"   ,    10,                                  1.6, FIELD_Y_SIZE - 0.9 - ROBOT_CENTER_X - 0.03, DIRECTION_FORWARD, PullFieldFire ,  (math.pi / 0.2,), False,    True))
        gm.add(goalmanager.Goal("FieldFireSE"   ,    10,                                  1.6, FIELD_Y_SIZE - 0.9 + ROBOT_CENTER_X + 0.03, DIRECTION_FORWARD, PushFieldFire , (-math.pi / 2.0,), False,    True))
        gm.add(goalmanager.Goal("HuntTheMammoth",    15,   0.3 + ROBOT_GYRATION_RADIUS + 0.03,                                       2.25, DIRECTION_FORWARD, HuntTheMammoth,              None, False,    True))

        yield AntiBlocking(True)
        yield Trigger(FIRE_FLIPPER_CLOSE)
        yield CalibratePosition()


    def on_start(self, packet):
        self.yield_at(90000, EndOfMatch())
        logger.log("Starting ...")
        self.send_packet(packets.ServoControl(*FIRE_FLIPPER_OPEN))
        yield MoveLineTo(RED_START_X, 1.35)
        self.send_packet(packets.ServoControl(*FIRE_FLIPPER_CLOSE))
        yield RotateTo(0.0)
        yield MoveLineTo(ROBOT_CENTER_X - 0.02, 1.35)
        yield Trigger(PAINT_1_FLIP_FLOP_START, PAINT_1_FLIP_FLOP_START)
        yield MoveLineTo(0.4, 1.35)
        yield Trigger(PAINT_1_FLIP_FLOP_STOP, PAINT_1_FLIP_FLOP_STOP)
        yield ExecuteGoals()




class CalibratePosition(statemachine.State):

    def on_enter(self):
        if IS_HOST_DEVICE_PC:
            yield DefinePosition(RED_START_X, RED_START_Y, RED_START_ANGLE)
        else:
            estimated_start_y = FIELD_Y_SIZE / 2.0
            yield DefinePosition(0.3 + ROBOT_CENTER_X, estimated_start_y, 0.0)
            yield MoveLineTo(RED_START_X, estimated_start_y)
            yield Rotate(math.pi / 2.0)
            yield SpeedControl(0.2)
            yield MoveLineTo(RED_START_X, 0.0, DIRECTION_BACKWARDS)
            yield SpeedControl()
            yield DefinePosition(None, ROBOT_CENTER_X, math.pi / 2.0)
        yield None




class HuntTheMammoth(statemachine.State):

    def on_enter(self):
        yield RotateTo(0.0)
        yield Trigger(GUN_FIRE)
        self.exit_reason = GOAL_DONE
        yield None




class PushBorderFire(statemachine.State):

    def __init__(self, angle):
        self.angle = angle


    def on_enter(self):
        yield RotateTo(self.angle)
        yield Trigger(FIRE_FLIPPER_OPEN)
        yield MoveLineRelative(0.05)
        yield MoveLineRelative(0.05, DIRECTION_BACKWARDS)
        yield Trigger(FIRE_FLIPPER_CLOSE)
        self.exit_reason = GOAL_DONE
        yield None




class PullBorderFire(statemachine.State):

    def __init__(self, angle):
        self.angle = angle


    def on_enter(self):
        yield RotateTo(self.angle)
        yield MoveLineRelative(0.05)
        yield Trigger(FIRE_FLIPPER_OPEN)
        yield MoveLineRelative(0.05, DIRECTION_BACKWARDS)
        yield Trigger(FIRE_FLIPPER_CLOSE)
        self.exit_reason = GOAL_DONE
        yield None




class PushFieldFire(statemachine.State):

    def __init__(self, angle):
        self.angle = angle


    def on_enter(self):
        yield RotateTo(self.angle)
        yield Trigger(FIRE_FLIPPER_OPEN)
        yield MoveLineRelative(0.05)
        yield Trigger(FIRE_FLIPPER_CLOSE)
        self.exit_reason = GOAL_DONE
        yield None




class PullFieldFire(statemachine.State):

    def __init__(self, angle):
        self.angle = angle


    def on_enter(self):
        yield RotateTo(self.angle)
        yield MoveLineRelative(0.05)
        yield Trigger(FIRE_FLIPPER_OPEN)
        yield MoveLineRelative(0.05, DIRECTION_BACKWARDS)
        yield Trigger(FIRE_FLIPPER_CLOSE)
        self.exit_reason = GOAL_DONE
        yield None




##################################################
# End of match




class EndOfMatch(statemachine.State):

    def on_enter(self):
        yield StopAll();
