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


FIRST_LINE_X = 0.94
SECOND_LINE_X = 1.19
THIRD_LINE_X = 1.45

FIRST_LINE_END_Y = 1.68
SECOND_LINE_END_Y = 1.5

TAKE_GLASS_DELTA_X = 0.08

START_X = FIRST_LINE_X + TAKE_GLASS_DELTA_X



class Main(statemachine.State):

    def on_enter(self):
        statemachine.StateMachine(self.event_loop, "barman", side = SIDE_LEFT)
        statemachine.StateMachine(self.event_loop, "barman", side = SIDE_RIGHT)
        self.fsm.cake = Cake()


    def on_device_ready(self, packet):
        yield AntiBlocking(True)
        yield CalibratePosition(START_X)


    def on_start(self, packet):
        detector = yield FetchCandleColors()
        self.log("First candles detection: {}".format(detector.colors))
        self.fsm.cake.update_with_detection(detector.colors)
        # yield GlassesSuperS()
        yield GlassesDirect()
        yield NavigateToCake(True)
        yield BlowCandlesOut()




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
# Alternate way of picking glasses




class GlassesAlternateFrancois(statemachine.State):

    def on_enter(self):
        #deplacement = commandMsg("MSG_MOVE_CURVE 1 1 1.3")
        # deplacement.addPose("0.85 0.85")
        # deplacement.addPose("1.15 0.75")
        # deplacement.addPose("1.65 0.85")
        # deplacement.addPose("1.83 1.05")
        # simulator_process.stdin.write(deplacement.cmdMsgGeneration())
        path = [
            (0.85, 0.85),
            (0.75, 1.15),
            (0.85, 1.65),
            (1.05, 1.83),
         ]
        # path = [
        #     (0.85, 0.85),
        #     (1.15, 0.75),
        #     (1.65, 0.85),
        #     (1.83, 1.05),
        # ]
        yield MoveCurve(1.3, path)

        # deplacement = commandMsg("MSG_ROTATE 0 1.57")
        # simulator_process.stdin.write(deplacement.cmdMsgGeneration())
        yield Rotate(math.pi/2)

        # deplacement = commandMsg("MSG_MOVE_LINE 1")
        # deplacement.addPose("1.83 1.2")
        # simulator_process.stdin.write(deplacement.cmdMsgGeneration())
        yield MoveLineTo( 1.2, 1.83)

        # deplacement = commandMsg("MSG_ROTATE 0 -2.9")
        # simulator_process.stdin.write(deplacement.cmdMsgGeneration())
        yield Rotate(-2.9)

        # deplacement = commandMsg("MSG_MOVE_LINE 1")
        # deplacement.addPose("0.8 1.0")
        # simulator_process.stdin.write(deplacement.cmdMsgGeneration())
        yield MoveLineTo(1.0, 0.8)

        # deplacement = commandMsg("MSG_ROTATE 0 1.37")
        # simulator_process.stdin.write(deplacement.cmdMsgGeneration())
        yield Rotate(1.37)

        # deplacement = commandMsg("MSG_MOVE_LINE 1")
        # deplacement.addPose("0.9 1.85")
        # simulator_process.stdin.write(deplacement.cmdMsgGeneration())
        yield MoveLineTo(1.85, 0.9)

        # deplacement = commandMsg("MSG_ROTATE 0 -1.17")
        # simulator_process.stdin.write(deplacement.cmdMsgGeneration())
        yield Rotate(-1.17)

        # deplacement = commandMsg("MSG_MOVE_ARC 1 1.5 2.0 0.6")
        # deplacement.addPose(str(- (4.0 * math.pi) / 8.0))
        # deplacement.addPose(str(- (0.6 * math.pi) / 8.0))
        # simulator_process.stdin.write(deplacement.cmdMsgGeneration())

        # deplacement = commandMsg("MSG_ROTATE 0 -1.57")
        # simulator_process.stdin.write(deplacement.cmdMsgGeneration())

        # deplacement = commandMsg("MSG_MOVE_ARC 1 1.2 2.0 0.9")
        # deplacement.addPose(str(- (2.0 * math.pi) / 8.0))
        # deplacement.addPose(str(- (4.0 * math.pi) / 8.0))
        # simulator_process.stdin.write(deplacement.cmdMsgGeneration())

        # deplacement = commandMsg("MSG_MOVE_LINE 1")
        # deplacement.addPose("0.2 1.1")
        # simulator_process.stdin.write(deplacement.cmdMsgGeneration())

        yield None


class GlassesDirect(statemachine.State):

    def on_enter(self):
        ohc = OpponentHandlingConfig(
            True,
            retries = 1
        )

        move = yield MoveLineTo( START_X, FIRST_LINE_END_Y, opponent_handling = ohc)
        move = yield Rotate(1.72, chained = move)

        #if we're in position, take a picture
        if move.exit_reason == REASON_DESTINATION_REACHED :
            detector = yield FetchCandleColors()
            self.log("Second candles detection: {}".format(detector.colors))
            self.fsm.cake.update_with_detection(detector.colors)

            yield Rotate(1.27)
            yield MoveLineTo( 0.97, 1.52, direction = DIRECTION_BACKWARDS)
            yield Rotate(0.84)
            yield MoveLineTo( 1.26, 1.81)

        else:
            x = SECOND_LINE_X + TAKE_GLASS_DELTA_X
            y = self.robot.virt.y
            yield LookAt(x,y)
            yield MoveLineTo(x,y)

        yield Rotate(-math.pi/2)
        yield MoveLineTo( 1.27, 1.16)


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
        return ordered




class NavigateToCake(Navigate):

    def __init__(self, team_side):
        x = 0.35
        if team_side:
            y = 1.0 - ROBOT_GYRATION_RADIUS + 0.05
            angle = 0.0
            direction = DIRECTION_BACKWARDS
        else:
            y = 2.0 + ROBOT_GYRATION_RADIUS - 0.05
            angle = math.pi
            direction = DIRECTION_FORWARDS
        Navigate.__init__(self, x, y, angle, direction)


    def create_path(self):
        path = Navigate.create_path(self)
        if len(path) != 0:
            path.append(Pose(ROBOT_CENTER_X, path[-1].y))
        return path




class BlowCandlesOut(statemachine.State):

    def on_enter(self):
        self.current = 0
        self.candles = self.fsm.cake.get_sorted_candles()
        if self.robot.pose.virt.y > FIELD_Y_SIZE / 2.0:
            # the robot is on the opposite site.
            reordered = [ candle for candle in reversed(self.candles) ]
            self.candles = reordered
            direction = DIRECTION_BACKWARDS
        else:
            direction = DIRECTION_FORWARDS
        self.side = SIDE_LEFT if self.robot.team == TEAM_BLUE else SIDE_RIGHT
        angles = [ candle.angle for candle in self.candles ]
        move = MoveArc(0.0, 1.5, 0.5 + ROBOT_GYRATION_RADIUS, angles, direction)
        move.on_waypoint_reached = self.on_waypoint_reached
        move.on_candle_kicker = self.on_candle_kicker
        yield move
        self.exit_reason = move.exit_reason == TRAJECTORY_DESTINATION_REACHED
        if self.exit_reason:
            yield MoveLineTo(0.5, self.robot.pose.virt.y, DIRECTION_BACKWARDS)
            self.send_packet(packets.CandleKicker(side = self.side, which = CANDLE_KICKER_UPPER, position = CANDLE_KICKER_POSITION_IDLE))
            self.send_packet(packets.CandleKicker(side = self.side, which = CANDLE_KICKER_LOWER, position = CANDLE_KICKER_POSITION_IDLE))
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

