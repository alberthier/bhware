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

        gm = self.robot.goal_manager
        gm.add(goalmanager.Goal("BorderFireRed1", 1, 2.0 - ROBOT_CENTER_X - 0.03, 1.3, DIRECTION_FORWARD, ToggleFire, None, False, True))


    def on_device_ready(self, packet):
        yield AntiBlocking(True)

        yield Trigger(GUN_LOAD, PAINT_1_HOLD, PAINT_2_HOLD, FIRE_FLIPPER_CLOSE)

        yield CalibratePosition()


    def on_start(self, packet):
        self.yield_at(90000, EndOfMatch())
        logger.log("Starting ...")
        self.send_packet(packets.ServoControl(*FIRE_FLIPPER_OPEN))
        yield MoveLineTo(RED_START_X, 1.2)
        self.send_packet(packets.ServoControl(*FIRE_FLIPPER_CLOSE))
        yield Rotate(0.0)
        yield MoveLineTo(ROBOT_CENTER_X - 0.02, 1.2)
        yield Trigger(PAINT_1_FLIP_FLOP_START, PAINT_1_FLIP_FLOP_START)
        yield MoveLineTo(0.4, 1.2)
        yield Trigger(PAINT_1_FLIP_FLOP_STOP, PAINT_1_FLIP_FLOP_STOP)
        yield ExecuteGoals()




class ToggleFire(statemachine.State):

    def on_enter(self):
        yield Trigger(FIRE_FLIPPER_OPEN)
        yield MoveLineRelative(-0.1, DIRECTION_BACKWARDS)
        self.exit_reason = GOAL_DONE
        yield None




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




##################################################
# End of match




class EndOfMatch(statemachine.State):

    def on_enter(self):
        yield StopAll();
