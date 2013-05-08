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

TAKE_GLASS_DELTA_X = 0.04

START_X = FIRST_LINE_X + TAKE_GLASS_DELTA_X

CAKE_ARC_RADIUS = 0.65

GIFT_Y_POS = [ 0.5, 1.1, 1.7, 2.3 ]
GIFT_X_POS = 1.77

GIFT_Y_DELTA = 0.05
GIFT_Y_POS = [ y + GIFT_Y_DELTA for y in GIFT_Y_POS ]

DEPOSIT_Y = 0.45


class Main(statemachine.State):

    def on_enter(self):
        statemachine.StateMachine(self.event_loop, "barman", side = SIDE_LEFT)
        statemachine.StateMachine(self.event_loop, "barman", side = SIDE_RIGHT)
        self.fsm.cake = Cake()

        gm  = goalmanager.GoalManager(self.fsm.event_loop)
        self.robot.goal_manager = gm

        # we may also handle each gift on its own

        # for i, x in enumerate(GIFT_Y_POS) :
        #     g = goalmanager.Goal(i, 1.0, x, 1.95, DIRECTION_FORWARDS, None)
        #     gm.harvesting_goals.append(g)

        gm.harvesting_goals.append(goalmanager.Goal("KICK_GIFTS", 1.0, GIFT_X_POS, 0.6, DIRECTION_FORWARDS,
                                                    KickGifts))
        gm.harvesting_goals.append(goalmanager.Goal("KICK_GIFTS", 1.0, GIFT_X_POS, GIFT_Y_POS[-1], DIRECTION_FORWARDS,
                                                    KickGifts))


    def on_device_ready(self, packet):
        yield AntiBlocking(True)
        yield CalibratePosition(START_X)


    def on_start(self, packet):
        detector = yield FetchCandleColors()
        self.log("First candles detection: {}".format(detector.colors))
        self.fsm.cake.update_with_detection(detector.colors)
        #yield GlassesSuperS()
        yield GlassesDirect()

        yield MoveRelative(.4)

        candles = self.fsm.cake.get_sorted_candles()
        self.log("Has {} candles to kick".format(len(candles)))
        if len(candles) == 0 or self.event_loop.get_elapsed_match_time() > 70.0:
            pass
        else:
            side = SIDE_LEFT if self.robot.team == TEAM_BLUE else SIDE_RIGHT
            nav = yield NavigateToCake(candles, CAKE_ARC_RADIUS)

            yield MoveRelative(-0.4, direction = DIRECTION_BACKWARDS)

            yield CandleKicker(side, CANDLE_KICKER_LOWER, CANDLE_KICKER_POSITION_IDLE)
            yield CandleKicker(side, CANDLE_KICKER_UPPER, CANDLE_KICKER_POSITION_IDLE)

        yield FindNextGoal()
        yield RingTheBell()





##################################################
# GIFTS KICKING
##################################################


class KickGifts(statemachine.State):
    def __init__(self, goal):
        self.goal = goal
        self.exit_reason = GOAL_FAILED

    def on_enter(self):
        gm = self.robot.goal_manager
        side = SIDE_LEFT if self.robot.team == TEAM_RED else SIDE_RIGHT

        yield LookAt(self.goal.x, self.goal.y)
        yield MoveLineTo(self.goal.x, self.goal.y)

        #TODO : handle being blocked on trajectory

        yield Rotate(math.pi/2)

        if self.robot.pose.virt.y < 1.5 :
            for y in GIFT_Y_POS:
                # y+=0.05
                direction = DIRECTION_BACKWARDS if y < self.robot.pose.virt.y else DIRECTION_FORWARDS
                yield MoveLineTo(GIFT_X_POS, y, direction)
                yield KickIt(side)
        else:
            for y in reversed(GIFT_Y_POS):
                # y+=0.05
                direction = DIRECTION_BACKWARDS if y < self.robot.pose.virt.y else DIRECTION_FORWARDS
                yield MoveLineTo(GIFT_X_POS, y, direction)
                yield KickIt(side)
            yield MoveRelative(0.05) # disengage

        self.exit_reason = GOAL_DONE

        yield None


class KickIt(statemachine.State):
    def __init__(self, side):
        self.side = side

    def on_enter(self):
        yield CandleKicker(self.side, CANDLE_KICKER_UPPER, CANDLE_KICKER_POSITION_UP)
        yield CandleKicker(self.side, CANDLE_KICKER_UPPER, CANDLE_KICKER_POSITION_IDLE)
        yield None




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
            y = self.robot.pose.virt.y
            yield LookAt(x,y)
            yield MoveLineTo(x,y)

        yield Rotate(-math.pi/2)
        yield MoveLineTo( 1.27, 1.16)


        yield None


class RingTheBell(statemachine.State):
    MIN_X = 0.27
    MIN_X_MIDDLE = 0.66
    MAX_X = 1.71


    def on_enter(self):
        x = self.robot.pose.virt.x
        y = self.robot.pose.virt.y

        if y >= 1.5 :
            x = max(x, self.MIN_X_MIDDLE)

        x = max(x, self.MIN_X)
        x = min(x, self.MAX_X)

        y = DEPOSIT_Y

        yield LookAt(x, y)
        # yield MoveLineTo(x, y)
        yield Navigate(x,y, DIRECTION_FORWARDS)
        yield DepositGlasses()
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
        #while len(ordered) != 0 and not ordered[0].to_blow:
            #del ordered[0]
        #while len(ordered) != 0 and not ordered[-1].to_blow:
            #del ordered[-1]
        return ordered




class NavigateToCake(statemachine.State):

    APPROACH_DISTANCE = 0.3

    def __init__(self, candles, cake_arc_radius):
        self.candles = candles
        self.cake_arc_radius = cake_arc_radius


    def on_enter(self):
        side = SIDE_LEFT if self.robot.team == TEAM_BLUE else SIDE_RIGHT
        yield Navigate(ROBOT_CENTER_X + 0.3, 1.5 - self.cake_arc_radius, DIRECTION_BACKWARDS)
        yield CandleKicker(side, CANDLE_KICKER_UPPER, CANDLE_KICKER_POSITION_UP)
        yield CandleKicker(side, CANDLE_KICKER_LOWER, CANDLE_KICKER_POSITION_UP)
        yield Rotate(0.0)
        yield MoveLineTo(ROBOT_CENTER_X, 1.5 - self.cake_arc_radius, DIRECTION_BACKWARDS)
        yield CandleKicker(side, CANDLE_KICKER_UPPER, CANDLE_KICKER_POSITION_KICK)
        yield CandleKicker(side, CANDLE_KICKER_LOWER, CANDLE_KICKER_POSITION_KICK)
        yield CandleKicker(side, CANDLE_KICKER_UPPER, CANDLE_KICKER_POSITION_UP)
        yield CandleKicker(side, CANDLE_KICKER_LOWER, CANDLE_KICKER_POSITION_UP)
        remaining_candles = []
        for candle in self.candles:
            if candle.name not in ["top1", "bottom1", "top8", "bottom12"]:
                remaining_candles.append(candle)
        yield BlowCandlesOut(remaining_candles, CAKE_ARC_RADIUS)
        yield None




    def compute_candle_pose(self, candle):
        start = Pose(self.cake_arc_radius * math.cos(candle.angle),
                     1.5 - self.cake_arc_radius * math.sin(candle.angle))
        approach = Pose(start.x + self.APPROACH_DISTANCE * math.sin(candle.angle),
                        start.y + self.APPROACH_DISTANCE * math.cos(candle.angle))
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

        angles = []
        for candle in self.candles:
            # 3.5 degrees is the difference between the candle kickers and the robot center.
            angles.append(candle.angle + math.radians(3.5))

        move = MoveArc(0.0, 1.5, self.cake_arc_radius, angles, direction)
        move.on_waypoint_reached = self.on_waypoint_reached
        move.on_candle_kicker = self.on_candle_kicker
        yield SpeedControl(0.4)
        yield move
        yield SpeedControl()
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

