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




class Candle:

    def __init__(self, name, angle, which, to_blow):
        self.name = name
        self.angle = angle
        self.which = which
        self.to_blow = to_blow




class Cake:

    def __init__(self):
        self.candles = {}
        for i in range(8):
            angle = math.radians(-90.0 + 11.25 + i * 22.5)
            name = "top" + str(i + 1)
            self.candles[name] = Candle(name, angle, CANDLE_KICKER_UPPER, i == 0)
        for i in range(12):
            angle = math.radians(-90.0 + 7.5 + i * 15.0)
            name = "bottom" + str(i + 1)
            candle = Candle(name, angle, CANDLE_KICKER_LOWER, i == 0)
            # White candles. Disable these two lines for final phases
            if i > 3 and i < 8:
                candle.to_blow = True
            self.candles[name] = candle


    def update_with_detection(self, detections):
        for name, to_blow in detections.items():
            if name in self.candles:
                self.candles[name].to_blow |= to_blow


    def get_sorted_candles(self):
        ordered = list(self.candles.values())
        ordered.sort(key = lambda candle: candle.angle)
        return ordered




class Main(statemachine.State):

    def on_enter(self):
        self.fsm.cake = Cake()


    def on_device_ready(self, packet):
        yield CalibratePosition()


    def on_start(self, packet):
        move = yield Navigate(1.5, 2.4)
        logger.log("elapsed: " + str(self.event_loop.get_elapsed_match_time()))
        logger.log(move.exit_reason)




class BlowCandlesOut(statemachine.State):

    def on_enter(self):
        self.current = 0
        self.candles = self.fsm.cake.get_sorted_candles()
        if self.robot.pose.virt.y > FIELD_Y_SIZE / 2.0:
            # the robot is on the opposite site.
            reordered = [ candle for candle in reversed(self.candles) ]
            self.candles = reordered
        if self.robot.pose.y > FIELD_Y_SIZE / 2.0:
            # Use real coordinates to select the correct side
            self.side = SIDE_RIGHT
        else:
            self.side = SIDE_LEFT
        angles = [ candle.angle for candle in self.candles ]
        move = MoveArc(0.0, 1.5, 0.5 + ROBOT_GYRATION_RADIUS, angles)
        move.on_waypoint_reached = self.on_waypoint_reached
        yield move
        self.exit_reason = move.exit_reason == TRAJECTORY_DESTINATION_REACHED
        if self.exit_reason:
            yield MoveLineTo(0.5, self.robot.pose.virt.y, DIRECTION_BACKWARDS)
            self.send_packet(packets.CandleKicker(self.side, CANDLE_KICKER_UPPER, CANDLE_KICKER_POSITION_IDLE))
            self.send_packet(packets.CandleKicker(self.side, CANDLE_KICKER_LOWER, CANDLE_KICKER_POSITION_IDLE))
        yield None


    def on_waypoint_reached(self, packet):
        candle = self.candles[self.current]
        self.current += 1
        if candle.to_blow:
            self.send_packet(packets.CandleKicker(self.side, candle.which, CANDLE_KICKER_POSITION_KICK))
    

    def on_candle_kicker(self, packet):
        if packet.position == CANDLE_KICKER_POSITION_KICK:
            self.send_packet(packets.CandleKicker(self.side, candle.which, CANDLE_KICKER_POSITION_UP))




class EndOfMatch(statemachine.State):

    def on_enter(self):
        yield StopAll();
        yield Pump(PUMP_ON)
        yield Timer(9000)
        yield Pump(PUMP_OFF)

