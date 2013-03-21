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
        logger.log("Main - on enter")

    def on_device_ready(self, packet):
        logger.log("Main - on device ready")
        #yield Timer(1000)
        #yield SetupPositionControl()
        #logger.log("Main - after pos control")
        yield DefinePosition()
        #yield Antiblocking(True)
        #yield Antiblocking(False)
        #yield BottomHolder(SIDE_RIGHT, MOVE_OPEN)
        #yield Lifter(SIDE_RIGHT, LIFTER_MOVE_MIDDLE)
        #yield Gripper(SIDE_RIGHT, MOVE_OPEN)
        #yield TopHolder(SIDE_RIGHT, MOVE_OPEN)
        #yield CandleKicker(SIDE_RIGHT, CANDLE_KICKER_UPPER, CANDLE_KICKER_POSITION_KICK)
        #yield GiftOpener(GIFT_OPENER_POSITION_RIGHT)
        #yield Pump(PUMP_ON)
        #yield FetchCandleColors()
        move = yield Rotate(DIRECTION_FORWARD, math.pi)
        move = yield MoveLine(DIRECTION_FORWARD, [Pose(1.5, 0.3)], move)
        move = yield MoveArc(DIRECTION_FORWARD, Pose(1.5, 0.0), 0.5, [0.0, math.pi], move)
        move = yield MoveCurve(DIRECTION_FORWARD, 0.0, [Pose(0.5, 1.0), Pose(0.3, 1.6)], move)
        logger.log(move.exit_reason)


class NewTest(statemachine.State):

    def on_enter(self):
        logger.log("NewTest - on enter")

    def on_exit(self):
        logger.log("NewTest - on exit")

    def on_start(self, packet):
        yield None




class Main2(statemachine.State):

    def on_device_ready(self, packet):
        self.switch_to_substate(commonstates.DefinePosition())


    def on_exit_substate(self, substate):
        self.switch_to_state(WaitStart())




class WaitStart(statemachine.State):

    def on_start(self, packet):
        yield commonstates.DefinePosition2()
        yield Test1()


    # def on_exit_substate(self, substate):
    #     self.switch_to_state(Test1())




class Test1(statemachine.State):

    def on_enter(self):
        #self.send_packet(packets.Rotate(angle = math.pi / 4))
        #self.send_packet(packets.MoveCurve(angle = 0.0, points = [Pose(RED_START_X + 0.2, 1.0), Pose(RED_START_X + 0.4, 1.5)]))
        #self.send_packet(packets.MoveArc(center = Pose(0.0, 0.0), radius = RED_START_X, points = [math.pi / 4, math.pi / 2]))
        self.walk = commonstates.TrajectoryWalk()
        self.walk.look_at_opposite(ROBOT_CENTER_X, 1.0 - ROBOT_CENTER_Y)
        self.walk.move_to(ROBOT_CENTER_X, 1.0 - ROBOT_CENTER_Y, DIRECTION_BACKWARD)
        self.walk.rotate_to(0.0)
        self.walk.follow_arc(DIRECTION_FORWARD, Pose(0.0, 1.5), 0.5 + ROBOT_Y_SIZE / 2.0, math.pi / 2.0 - math.pi / 12.0)
        #self.walk.wait_for(commonstates.FetchCandleColors())
        #self.walk.wait_for(commonstates.CandleKicker(SIDE_RIGHT, CANDLE_KICKER_LOWER, CANDLE_KICKER_POSITION_UP))
        #self.walk.wait_for(commonstates.CandleKicker(SIDE_RIGHT, CANDLE_KICKER_LOWER, CANDLE_KICKER_POSITION_KICK))
        #self.walk.wait_for(commonstates.CandleKicker(SIDE_RIGHT, CANDLE_KICKER_LOWER, CANDLE_KICKER_POSITION_IDLE))
        #self.walk.wait_for(commonstates.GiftOpener(GIFT_OPENER_POSITION_LEFT))
        #self.walk.wait_for(commonstates.GiftOpener(GIFT_OPENER_POSITION_IDLE))
        #self.walk.wait_for(commonstates.GiftOpener(GIFT_OPENER_POSITION_RIGHT))
        #self.walk.look_at(1.5, 1.6)
        #self.walk.move_to(1.5, 1.6)
        #self.walk.on_glass_present = self.on_glass_present
        self.switch_to_substate(self.walk)
        #self.first = True


    #def on_glass_present(self, packet):
        #self.send_packet(packets.Stop())


    def on_exit_substate(self, substate):
        return
        if self.first:
            self.first = False
            self.send_packet(packets.Nipper(side=SIDE_LEFT, move=MOVE_CLOSE))
            self.walk = commonstates.TrajectoryWalk()
            self.walk.look_at(1.0, 1.0)
            self.walk.move_to(1.0, 1.0)
            self.switch_to_substate(self.walk)



class EndOfMatch(statemachine.State):

    pass
