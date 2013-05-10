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

FIRST_LINE_END_Y = 1.95
SECOND_LINE_END_Y = 1.5

TAKE_GLASS_DELTA_X = 0.04

START_X = FIRST_LINE_X + TAKE_GLASS_DELTA_X
SECOND_SHOT_ANGLE = math.radians(150.0)

CAKE_ARC_RADIUS = 0.65

GIFT_Y_POS = [ 0.6, 1.1, 1.7, 2.3 ]
GIFT_X_POS = 1.77

GIFT_Y_DELTA = 0.08

DEPOSIT_Y = 0.45




class GiftGoal(goalmanager.Goal):

    def __init__(self, forward):
        super().__init__("KICK_GIFTS", 1.0, GIFT_X_POS, 0.0, DIRECTION_BACKWARDS, KickGifts)
        self.forward = True
        self.update()


    def update(self):
        if len(GIFT_Y_POS) != 0:
            if self.forward:
                self.y = GIFT_Y_POS[0]
                logger.log("Update forward Gift Y to {}".format(self.y))
            else:
                self.y = GIFT_Y_POS[-1]
                logger.log("Update backward Gift Y to {}".format(self.y))




class Main(statemachine.State):

    def on_enter(self):
        lb = statemachine.StateMachine(self.event_loop, "barman", side = SIDE_LEFT)
        rb = statemachine.StateMachine(self.event_loop, "barman", side = SIDE_RIGHT)

        gm  = goalmanager.GoalManager(self.fsm.event_loop)
        self.robot.goal_manager = gm

        # we may also handle each gift on its own

        # for i, x in enumerate(GIFT_Y_POS) :
        #     g = goalmanager.Goal(i, 1.0, x, 1.95, DIRECTION_FORWARDS, None)
        #     gm.harvesting_goals.append(g)

        self.fsm.forward_way_gifts = GiftGoal(True)
        gm.harvesting_goals.append(self.fsm.forward_way_gifts)
        self.fsm.backward_way_gifts = GiftGoal(False)
        gm.harvesting_goals.append(self.fsm.backward_way_gifts)
        gm.harvesting_goals.append(goalmanager.Goal("CAKE", 1.1, ROBOT_CENTER_X + 0.3, 1.5 - CAKE_ARC_RADIUS, DIRECTION_BACKWARDS,
                                                    PrepareCakeMove))

        deposit_glasses_x_list = [ (0.32, 2.45), (0.6, 2.5), (1.0, 2.55)]

        for x, priority in deposit_glasses_x_list :

            deposit_glasses_goal = goalmanager.GlassDepositGoal('GLASSES_DEPOSIT', priority, x, 0.5,
                                                                DIRECTION_FORWARDS, RingTheBell, shared = False)

            deposit_glasses_goal.barmen = [lb, rb]

            gm.harvesting_goals.append(deposit_glasses_goal)


    def on_device_ready(self, packet):
        yield AntiBlocking(True)
        yield CalibratePosition(START_X)


    def on_start(self, packet):
        self.fsm.cake = Cake(packet.team)
        detector = yield FetchCandleColors()
        self.log("First candles detection: {}".format(detector.colors))
        self.fsm.cake.update_with_detection(detector.colors)
        yield GlassesDirect()
        yield FindNextGoal()
        yield Navigate(ROBOT_CENTER_X + 0.3, 1.5 - CAKE_ARC_RADIUS)
        yield PrepareCakeMove(None)





##################################################
# GIFTS KICKING
##################################################


class KickGifts(statemachine.State):
    def __init__(self, goal):
        self.goal = goal
        self.exit_reason = GOAL_FAILED

    def on_enter(self):
        side = SIDE_LEFT if self.robot.team == TEAM_RED else SIDE_RIGHT

        ohc = OpponentHandlingConfig(
            True,
            retries = 2
        )


        move = yield Rotate(math.pi/2)

        if self.robot.pose.virt.y < 1.5 :
            while len(GIFT_Y_POS) != 0:
                y = GIFT_Y_POS[0]

                if y < 0.7 :
                    y = 0.5

                direction = DIRECTION_BACKWARDS if y < self.robot.pose.virt.y else DIRECTION_FORWARDS
                move = yield MoveLineTo(GIFT_X_POS, y + GIFT_Y_DELTA, direction, chained = move, opponent_handling = ohc )

                if move.exit_reason != TRAJECTORY_DESTINATION_REACHED :
                    break
                yield KickIt(side)
                del GIFT_Y_POS[0]

        else:
            while len(GIFT_Y_POS) != 0:
                y = GIFT_Y_POS[-1]
                # y+=0.05
                direction = DIRECTION_BACKWARDS if y < self.robot.pose.virt.y else DIRECTION_FORWARDS
                move = yield MoveLineTo(GIFT_X_POS, y - GIFT_Y_DELTA, direction, chained = move, opponent_handling = ohc)
                if move.exit_reason != TRAJECTORY_DESTINATION_REACHED :
                    break
                yield KickIt(side)
                del GIFT_Y_POS[-1]
            yield MoveRelative(0.05, chained = move) # disengage

        # for the moment, there's no distinct handling for each gift

        # degagement - match 4

        #if self.robot.pose.virt.y > 1.5 :
            #new_pos_x = self.robot.pose.virt.x
            #new_pos_y = 1.5

            #yield MoveLineTo(new_pos_x, new_pos_y, DIRECTION_BACKWARDS)

        # degagement - vers le haut

        # yield Rotate(math.pi)
        # yield MoveRelative(0.5)

        if len(GIFT_Y_POS) == 0:
            self.exit_reason = GOAL_DONE
        else:
            self.fsm.forward_way_gifts.update()
            self.fsm.backward_way_gifts.update()

        yield None




class KickIt(statemachine.State):
    def __init__(self, side):
        self.side = side

    def on_enter(self):
        yield CandleKicker(self.side, CANDLE_KICKER_UPPER, CANDLE_KICKER_POSITION_UP)
        yield CandleKicker(self.side, CANDLE_KICKER_UPPER, CANDLE_KICKER_POSITION_IDLE)
        yield None




class GlassesDirect(statemachine.State):

    def on_enter(self):
        ohc = OpponentHandlingConfig(
            True,
            retries = 1
        )

        points = [ (START_X, ROBOT_CENTER_Y + 0.1), (1.19, 1.12), (1.23, 1.3), (1.17, 1.65),
                                        (0.95, 1.83) ]


        yield SpeedControl(0.4)
        move = yield MoveCurve( 2.87, points, opponent_handling = ohc)
        yield SpeedControl()
        move = yield Rotate(math.pi, chained = move)
        move = yield MoveLineTo(0.8, 1.83, chained = move, opponent_handling =  ohc)
        move = yield Rotate(-1.33, chained = move)
        move = yield MoveLineTo(1.0, 0.8, chained = move, opponent_handling = ohc)

        yield move

        yield None


class RingTheBell(statemachine.State):
    MIN_X = 0.27
    MIN_X_MIDDLE = 0.66
    MAX_X = 1.71

    def __init__(self, goal):
        self.goal = goal
        self.exit_reason = GOAL_FAILED


    def on_enter(self):
        yield Rotate(-math.pi/2)
        yield MoveRelative(0.1)
        yield DepositGlasses()
        yield MoveRelative(-0.2, DIRECTION_BACKWARDS)

        self.goal.used = True

        self.exit_reason = GOAL_DONE
        yield None





##################################################
# Blow candles out




class Candle:

    def __init__(self, cake, name, angle, which, to_blow):
        cake.candles[name] = self
        self.name = name
        self.angle = angle
        self.which = which
        self.to_blow = to_blow




class Cake:

    def __init__(self, team):
        self.candles = {}
        self.symetry = {
                "top1"   : "top8",
                "top2"   : "top7",
                "top3"   : "top6",
                "top4"   : "top5",
                "bottom1": "bottom12",
                "bottom2": "bottom11",
                "bottom3": "bottom10",
                "bottom4": "bottom9",
                "bottom5": "bottom8",
                "bottom6": "bottom7",
        }

        if team == TEAM_BLUE:
            # BLUE Top candles
            Candle(self, "top1", math.radians(-90.0 + 11.25 + 0.0 * 22.5 + 5.0), CANDLE_KICKER_UPPER, True)
            Candle(self, "top2", math.radians(-90.0 + 11.25 + 1.0 * 22.5 + 5.0), CANDLE_KICKER_UPPER, False)
            Candle(self, "top3", math.radians(-90.0 + 11.25 + 2.0 * 22.5 + 5.0), CANDLE_KICKER_UPPER, False)
            Candle(self, "top4", math.radians(-90.0 + 11.25 + 3.0 * 22.5 + 5.0), CANDLE_KICKER_UPPER, False)
            Candle(self, "top5", math.radians(-90.0 + 11.25 + 4.0 * 22.5 + 7.0), CANDLE_KICKER_UPPER, False)
            Candle(self, "top6", math.radians(-90.0 + 11.25 + 5.0 * 22.5 + 8.0), CANDLE_KICKER_UPPER, False)
            Candle(self, "top7", math.radians(-90.0 + 11.25 + 6.0 * 22.5 + 9.0), CANDLE_KICKER_UPPER, False)
            Candle(self, "top8", math.radians(-90.0 + 11.25 + 7.0 * 22.5 + 9.0), CANDLE_KICKER_UPPER, False)

            # BLUE Bottom candles
            Candle(self, "bottom1"  , math.radians(-90.0 + 7.5 +  0.0 * 15.0 + 5.0), CANDLE_KICKER_LOWER, True)
            Candle(self, "bottom2"  , math.radians(-90.0 + 7.5 +  1.0 * 15.0 + 5.0), CANDLE_KICKER_LOWER, False)
            Candle(self, "bottom3"  , math.radians(-90.0 + 7.5 +  2.0 * 15.0 + 5.0), CANDLE_KICKER_LOWER, False)
            Candle(self, "bottom4"  , math.radians(-90.0 + 7.5 +  3.0 * 15.0 + 5.0), CANDLE_KICKER_LOWER, False)
            Candle(self, "bottom5"  , math.radians(-90.0 + 7.5 +  4.0 * 15.0 + 5.0), CANDLE_KICKER_LOWER, True)
            Candle(self, "bottom6"  , math.radians(-90.0 + 7.5 +  5.0 * 15.0 + 5.0), CANDLE_KICKER_LOWER, True)
            Candle(self, "bottom7"  , math.radians(-90.0 + 7.5 +  6.0 * 15.0 + 7.0), CANDLE_KICKER_LOWER, True)
            Candle(self, "bottom8"  , math.radians(-90.0 + 7.5 +  7.0 * 15.0 + 7.0), CANDLE_KICKER_LOWER, True)
            Candle(self, "bottom9"  , math.radians(-90.0 + 7.5 +  8.0 * 15.0 + 7.0), CANDLE_KICKER_LOWER, False)
            Candle(self, "bottom10" , math.radians(-90.0 + 7.5 +  9.0 * 15.0 + 7.0), CANDLE_KICKER_LOWER, False)
            Candle(self, "bottom11" , math.radians(-90.0 + 7.5 + 10.0 * 15.0 + 7.0), CANDLE_KICKER_LOWER, False)
            Candle(self, "bottom12" , math.radians(-90.0 + 7.5 + 11.0 * 15.0 + 7.0), CANDLE_KICKER_LOWER, False)
        else:
            # RED Top candles
            Candle(self, "top1", math.radians(-90.0 + 11.25 + 0.0 * 22.5 + 5.0), CANDLE_KICKER_UPPER, True)
            Candle(self, "top2", math.radians(-90.0 + 11.25 + 1.0 * 22.5 + 5.0), CANDLE_KICKER_UPPER, False)
            Candle(self, "top3", math.radians(-90.0 + 11.25 + 2.0 * 22.5 + 5.0), CANDLE_KICKER_UPPER, False)
            Candle(self, "top4", math.radians(-90.0 + 11.25 + 3.0 * 22.5 + 5.0), CANDLE_KICKER_UPPER, False)
            Candle(self, "top5", math.radians(-90.0 + 11.25 + 4.0 * 22.5 + 5.0), CANDLE_KICKER_UPPER, False)
            Candle(self, "top6", math.radians(-90.0 + 11.25 + 5.0 * 22.5 + 5.0), CANDLE_KICKER_UPPER, False)
            Candle(self, "top7", math.radians(-90.0 + 11.25 + 6.0 * 22.5 + 6.0), CANDLE_KICKER_UPPER, False)
            Candle(self, "top8", math.radians(-90.0 + 11.25 + 7.0 * 22.5 + 6.0), CANDLE_KICKER_UPPER, False)

            # RED Bottom candles
            Candle(self, "bottom1"  , math.radians(-90.0 + 7.5 +  0.0 * 15.0 + 5.0), CANDLE_KICKER_LOWER, True)
            Candle(self, "bottom2"  , math.radians(-90.0 + 7.5 +  1.0 * 15.0 + 5.0), CANDLE_KICKER_LOWER, False)
            Candle(self, "bottom3"  , math.radians(-90.0 + 7.5 +  2.0 * 15.0 + 5.0), CANDLE_KICKER_LOWER, False)
            Candle(self, "bottom4"  , math.radians(-90.0 + 7.5 +  3.0 * 15.0 + 5.0), CANDLE_KICKER_LOWER, False)
            Candle(self, "bottom5"  , math.radians(-90.0 + 7.5 +  4.0 * 15.0 + 5.0), CANDLE_KICKER_LOWER, True)
            Candle(self, "bottom6"  , math.radians(-90.0 + 7.5 +  5.0 * 15.0 + 5.0), CANDLE_KICKER_LOWER, True)
            Candle(self, "bottom7"  , math.radians(-90.0 + 7.5 +  6.0 * 15.0 + 5.0), CANDLE_KICKER_LOWER, True)
            Candle(self, "bottom8"  , math.radians(-90.0 + 7.5 +  7.0 * 15.0 + 3.0), CANDLE_KICKER_LOWER, True)
            Candle(self, "bottom9"  , math.radians(-90.0 + 7.5 +  8.0 * 15.0 + 3.0), CANDLE_KICKER_LOWER, False)
            Candle(self, "bottom10" , math.radians(-90.0 + 7.5 +  9.0 * 15.0 + 3.0), CANDLE_KICKER_LOWER, False)
            Candle(self, "bottom11" , math.radians(-90.0 + 7.5 + 10.0 * 15.0 + 2.0), CANDLE_KICKER_LOWER, False)
            Candle(self, "bottom12" , math.radians(-90.0 + 7.5 + 11.0 * 15.0 + 2.0), CANDLE_KICKER_LOWER, False)


    def update_with_detection(self, detections):
        for name, to_blow in detections.items():
            if name in self.symetry:
                symetric_name = self.symetry[name]
            else:
                symetric_name = ""
            if name in self.candles:
                candle = self.candles[name]
                if not candle.to_blow:
                    candle.to_blow = to_blow
            if symetric_name in self.candles:
                candle = self.candles[symetric_name]
                if not candle.to_blow:
                    candle.to_blow = not to_blow


    def get_sorted_candles(self):
        ordered = list(self.candles.values())
        ordered.sort(key = lambda candle: candle.angle)
        return ordered




class PrepareCakeMove(statemachine.State):

    def __init__(self, goal):
        self.goal = goal
        self.exit_reason = GOAL_FAILED


    def on_enter(self):

        candles = self.fsm.cake.get_sorted_candles()
        self.log("Has {} candles to kick".format(len(candles)))
        if len(candles) == 0:
            yield None
            return

        side = SIDE_LEFT if self.robot.team == TEAM_BLUE else SIDE_RIGHT
        if self.robot.team == TEAM_RED:
            radius = CAKE_ARC_RADIUS + 0.015
        else:
            radius = CAKE_ARC_RADIUS

        yield Rotate(0.0)
        yield CandleKicker(side, CANDLE_KICKER_UPPER, CANDLE_KICKER_POSITION_UP)
        self.send_packet(packets.CandleKicker(side = side, which = CANDLE_KICKER_LOWER, position = CANDLE_KICKER_POSITION_UP))
        yield MoveLineTo(ROBOT_CENTER_X, 1.5 - CAKE_ARC_RADIUS, DIRECTION_BACKWARDS)
        yield CandleKicker(side, CANDLE_KICKER_UPPER, CANDLE_KICKER_POSITION_KICK)
        yield CandleKicker(side, CANDLE_KICKER_LOWER, CANDLE_KICKER_POSITION_KICK)
        yield CandleKicker(side, CANDLE_KICKER_UPPER, CANDLE_KICKER_POSITION_UP)
        yield CandleKicker(side, CANDLE_KICKER_LOWER, CANDLE_KICKER_POSITION_UP)
        remaining_candles = []
        for candle in candles:
            if candle.name not in ["top1", "bottom1", "top8", "bottom12"]:
                remaining_candles.append(candle)
        if len(remaining_candles) != 0:
            bco = yield BlowCandlesOut(remaining_candles, radius)
            self.exit_reason = bco.exit_reason
        else:
            self.exit_reason = GOAL_DONE
        self.send_packet(packets.CandleKicker(side = side, which = CANDLE_KICKER_LOWER, position = CANDLE_KICKER_POSITION_IDLE))
        self.send_packet(packets.CandleKicker(side = side, which = CANDLE_KICKER_UPPER, position = CANDLE_KICKER_POSITION_IDLE))
        yield None




class BlowCandlesOut(statemachine.State):

    def __init__(self, candles, cake_arc_radius):
        self.candles = candles
        self.cake_arc_radius = cake_arc_radius
        self.exit_reason = GOAL_FAILED


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
        yield SpeedControl(0.25)
        yield move
        yield SpeedControl()
        yield MoveRelative(-0.3, DIRECTION_BACKWARDS)
        if move.exit_reason != TRAJECTORY_DESTINATION_REACHED:
            yield None
            return
        if self.candles[-1].to_blow:
            yield CandleKicker(self.side, self.candles[-1].which, CANDLE_KICKER_POSITION_KICK)
            yield CandleKicker(self.side, self.candles[-1].which, CANDLE_KICKER_POSITION_UP)
        self.exit_reason = GOAL_DONE
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
        yield Timer(5000)
        yield Pump(PUMP_OFF)

