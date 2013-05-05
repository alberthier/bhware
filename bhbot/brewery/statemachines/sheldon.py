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

    CAKE_ARC_RADIUS = 0.5 + ROBOT_GYRATION_RADIUS

    def on_enter(self):
        statemachine.StateMachine(self.event_loop, "barman", side = SIDE_LEFT)
        statemachine.StateMachine(self.event_loop, "barman", side = SIDE_RIGHT)
        self.fsm.cake = Cake()


    def on_device_ready(self, packet):
        yield AntiBlocking(True)
        yield CalibratePosition()


    def on_start(self, packet):
        detector = yield FetchCandleColors()
        self.log("First candles detection: {}".format(detector.colors))
        self.fsm.cake.update_with_detection(detector.colors)
        #yield GlassesSuperS()
        while True:
            candles = self.fsm.cake.get_sorted_candles()
            if len(candles) == 0 or self.event_loop.get_elapsed_match_time() < 70.0:
                break
            else:
                side = SIDE_LEFT if self.robot.team == TEAM_BLUE else SIDE_RIGHT
                nav = yield NavigateToCake(candles, CAKE_ARC_RADIUS)
                yield BlowCandlesOut(candles, CAKE_ARC_RADIUS)
                yield MoveRelative(0.1, -nav.direction)
                yield CandleKicker(side, CANDLE_KICKER_UPPER, CANDLE_KICKER_POSITION_UP)
                yield CandleKicker(side, CANDLE_KICKER_LOWER, CANDLE_KICKER_POSITION_UP)
                break




##################################################
# S curve to pick up the glasses




class GlassesSuperS(statemachine.State):

    def on_enter(self):
        glasses = [(0.95, 0.90),
                   (1.20, 1.05),
                   (0.95, 1.20),
                   (1.20, 1.35),
                   (1.20, 1.65),
                   (0.95, 1.80)]
        path = []
        for x, y in glasses:
            xoffset = 0.05
            if x < 1.0:
                x += xoffset
            else:
                x -= xoffset
            y -= 0.128
            path.append((x, y))
        yield MoveCurve(math.pi /2.0, path)
        yield None




##################################################
# Blow candles out




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
        while len(ordered) != 0 and not ordered[0].to_blow:
            del ordered[0]
        while len(ordered) != 0 and not ordered[-1].to_blow:
            del ordered[-1]
        return ordered




class NavigateToCake(statemachine.State):

    APPROACH_DISTANCE = 0.3

    def __init__(self, candles, cake_arc_radius):
        self.candles = candles
        self.cake_arc_radius = cake_arc_radius


    def on_enter(self):
        (my_approach, my_start) = self.compute_candle_pose(candles[0])
        (opponent_approach, opponent_start) = self.compute_candle_pose(candles[-1])
        (my_cost, my_path) = self.event_loop.map.route(self.robot.pose, team_approach)
        (opponent_cost, opponent_path) = self.event_loop.map.route(self.robot.pose, team_approach)
        if my_cost is None:
            cost = opponent_cost
            path = opponent_path
            start = opponent_start
            direction = DIRECTION_FORWARDS
        elif opponent_cost is None:
            cost = my_cost
            path = my_path
            start = my_start
            direction = DIRECTION_BACKWARDS
        elif my_cost < opponent_cost:
            cost = my_cost
            path = my_path
            start = my_start
            direction = DIRECTION_BACKWARDS
        else:
            cost = opponent_cost
            path = opponent_path
            start = opponent_start
            direction = DIRECTION_FORWARDS
        if cost is None:
            yield None
            return
        yield FollowPath(path, direction)
        yield CandleKicker(side, CANDLE_KICKER_UPPER, CANDLE_KICKER_POSITION_UP)
        yield CandleKicker(side, CANDLE_KICKER_LOWER, CANDLE_KICKER_POSITION_UP)
        if direction == DIRECTION_FORWARDS:
            yield LookAt(start.x, start.y)
        else:
            yield LookAtOpposite(start.x, start.y)
        yield MoveLineTo([start], direction)
        yield None


    def compute_candle_pose(self, candle):
        start = Pose(self.cake_arc_radius * math.cos(candle.angle),
                     1.5 - self.cake_arc_radius * math.sin(candle.angle))
        approach = Pose(start.x + APPROACH_DISTANCE * math.sin(candle.angle),
                        start.y + APPROACH_DISTANCE * math.cos(candle.angle))
        return (approach, start)




class BlowCandlesOut(statemachine.State):


    def __init__(self, candles, cake_arc_radius):
        self.candles = candles
        self.cake_arc_radius = cake_arc_radius


    def on_enter(self):
        self.current = 0
        if self.robot.pose.virt.y > FIELD_Y_SIZE / 2.0:
            # the robot is on the opposite site.
            reordered = [ candle for candle in reversed(self.candles) ]
            self.candles = reordered
            direction = DIRECTION_BACKWARDS
        else:
            direction = DIRECTION_FORWARDS
        self.side = SIDE_LEFT if self.robot.team == TEAM_BLUE else SIDE_RIGHT

        angles = [ candle.angle for candle in self.candles ]
        move = MoveArc(0.0, 1.5, self.cake_arc_radius, angles, direction)
        move.on_waypoint_reached = self.on_waypoint_reached
        move.on_candle_kicker = self.on_candle_kicker
        yield move
        self.exit_reason = move.exit_reason == TRAJECTORY_DESTINATION_REACHED
        yield None


    def on_waypoint_reached(self, packet):
        candle = self.candles[self.current]
        self.current += 1
        if candle.to_blow:
            self.send_packet(packets.CandleKicker(side = self.side, which = candle.which, position = CANDLE_KICKER_POSITION_KICK))


    def on_candle_kicker(self, packet):
        if packet.position == CANDLE_KICKER_POSITION_KICK:
            self.send_packet(packets.CandleKicker(side = self.side, which = packet.which, position = CANDLE_KICKER_POSITION_UP))




##################################################
# End of match - Baloon




class EndOfMatch(statemachine.State):

    def on_enter(self):
        yield StopAll();
        yield Pump(PUMP_ON)
        yield Timer(9000)
        yield Pump(PUMP_OFF)

