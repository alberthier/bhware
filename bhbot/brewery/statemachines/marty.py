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

import statemachines.testscommon as testscommon
import statemachines.testsmarty as testsmarty


BORDER_FIRE_DIST = 0.15
FIELD_FIRE_DIST  = 0.15
MAMMOTH_HUNT_Y   = 2.45 




class DisabledGoal(goalmanager.Goal):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.status = GOAL_DISABLED




class ProtectionGoal(goalmanager.Goal):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @property
    def status(self):
        if self.goal_manager.is_done("DepositFires_Mine") :
            return GOAL_AVAILABLE
        return GOAL_DISABLED

    @status.setter
    def status(self, value):
        pass




class NoBotherGoal(goalmanager.Goal):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @property
    def status(self):
        if self.goal_manager.is_done("DepositFires_Mine") :
            return GOAL_DONE
        return GOAL_AVAILABLE

    @status.setter
    def status(self, value):
        pass




class CalibrationGoal(goalmanager.Goal):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.cal_x = 0.0
        self.cal_y = 0.0
        self.cal_angle = 0.0

    def get_state(self):
        if self.cal_y > 0.0 :
            return CalibrateAxis("y", self.cal_y, self.cal_angle)




class CalibrateAxis(statemachine.State):

    def __init__(self, axis : str, position : float, angle : float):
        self.angle = angle
        self.position = position
        self.axis = axis

    def on_enter(self):

        yield RotateTo(self.angle)

        delta_pos = 0.2
        margin_pos = 0.05
        margin_angle = math.pi / 6

        if self.axis == "x":

            if self.robot.pose.virt.angle > math.pi/2 or self.robot.pose.virt.angle < -math.pi/2 :
                target_x = self.position + delta_pos
            else :
                target_x = self.position - delta_pos

            yield SpeedControl(0.2)

            move = yield MoveLineTo(target_x, self.robot.pose.virt.y, direction=DIRECTION_BACKWARDS)

            yield SpeedControl()

            if tools.compare_angles(self.robot.pose.virt.angle, self.angle, margin_angle) :
                yield DefinePosition(self.position, None, angle=self.angle)
            else :
                logger.log("No resettle because angle seems to be wrong (expected: {}, got: {}".format(self.robot.pose.virt.angle, self.angle))

            yield MoveLineRelative(delta_pos, direction = DIRECTION_FORWARD)

        if self.axis == "y":
            if self.robot.pose.virt.angle > 0.0 :
                target_y = self.position - delta_pos
            else :
                target_y = self.position + delta_pos

            yield SpeedControl(0.2)

            move = yield MoveLineTo(self.robot.pose.virt.x, target_y, direction=DIRECTION_BACKWARDS)

            yield SpeedControl()

            if tools.compare_angles(self.robot.pose.virt.angle, self.angle, margin_angle) :
                yield DefinePosition(None, self.position, angle=self.angle)
            else:
                logger.log("No resettle because angle seems to be wrong (expected: {}, got: {}".format(self.robot.pose.virt.angle, self.angle))

            yield MoveLineRelative(delta_pos, direction = DIRECTION_FORWARD)

        self.exit_reason = GOAL_DONE
        yield None







class Main(statemachine.State):

    def on_enter(self):
        statemachine.StateMachine(self.event_loop, "interbot")
        statemachine.StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_MAIN)
        statemachine.StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_SECONDARY)
        statemachine.StateMachine(self.event_loop, "relaytoggler")


    def on_device_ready(self, packet):
        gm = self.robot.goal_manager

        # calibration_y1 = CalibrationGoal("ReCalibrateY"   ,    14,                                             0.20,                                           1.78 , DIRECTION_BACKWARDS,  None, (-math.pi / 2.0,), False,    True)
        # calibration_y1.cal_angle = -math.pi/2
        # calibration_y1.cal_y = 1.90 - SECONDARY_ROBOT_CENTER_Y
        #
        # calibration_y2 = CalibrationGoal("ReCalibrateY"   ,    14,                                             0.20,                                    sym_y(1.78) , DIRECTION_BACKWARDS,  None, (-math.pi / 2.0,), False,    True)
        # calibration_y2.cal_angle = math.pi/2
        # calibration_y2.cal_y = 1.10 + SECONDARY_ROBOT_CENTER_Y
        #
        # calibration_x1 = CalibrationGoal("ReCalibrateX"   ,    14,                                             0.42,                                           1.0 , DIRECTION_BACKWARDS,  None,              (0.0,), False,    True)
        # calibration_x1.cal_angle = 0.0
        # calibration_x1.cal_x = 0.9 + SECONDARY_ROBOT_CENTER_Y

        #                      |       ID       |Weight|                            X                    |                           Y                     |     Direction    |     State     | Ctor parameters  |Shared|Navigate|
        # gm.add(goalmanager.Goal("BorderFireW"   ,     2,                                              0.8,               ROBOT_CENTER_X + BORDER_FIRE_DIST , DIRECTION_FORWARD, PullBorderFire, (-math.pi / 2.0,), False,    True))
        gm.add(goalmanager.Goal("BorderFireSW"  ,    10, FIELD_X_SIZE - ROBOT_CENTER_X - BORDER_FIRE_DIST,                                             1.3 , DIRECTION_FORWARD, PushBorderFire,            (0.0,), False,    True))
        gm.add(goalmanager.Goal("BorderFireSE"  ,    10, FIELD_X_SIZE - ROBOT_CENTER_X - BORDER_FIRE_DIST,                                       sym_y(1.3), DIRECTION_FORWARD, PullBorderFire,            (0.0,), False,    True))
        gm.add(goalmanager.Goal("BorderFireE"   ,    10,                                              0.8,         sym_y(ROBOT_CENTER_X + BORDER_FIRE_DIST), DIRECTION_FORWARD, PushBorderFire,  (math.pi / 2.0,), False,    True))
        # gm.add(goalmanager.Goal("FieldFireW"    ,     7,           1.1 - ROBOT_CENTER_X - FIELD_FIRE_DIST,                                             0.4 , DIRECTION_FORWARD, PushFieldFire ,            (0.0,), False,    True))
        # gm.add(goalmanager.Goal("FieldFireW"    ,     7,           1.1 + ROBOT_CENTER_X + FIELD_FIRE_DIST,                                             0.4 , DIRECTION_FORWARD, PullFieldFire ,        (math.pi,), False,    True))
        # gm.add(goalmanager.Goal("FieldFireSW"   ,     7,                                              1.6,          0.9 - ROBOT_CENTER_X - FIELD_FIRE_DIST , DIRECTION_FORWARD, PullFieldFire ,  (math.pi / 0.2,), False,    True))
        # gm.add(goalmanager.Goal("FieldFireSW"   ,     7,                                              1.6,          0.9 + ROBOT_CENTER_X + FIELD_FIRE_DIST , DIRECTION_FORWARD, PushFieldFire , (-math.pi / 2.0,), False,    True))
        gm.add(goalmanager.Goal("FieldFireSE"   ,    10,                                              1.6,    sym_y(0.9 + ROBOT_CENTER_X + FIELD_FIRE_DIST), DIRECTION_FORWARD, PullFieldFire ,  (math.pi / 2.0,), False,    True))
        gm.add(goalmanager.Goal("FieldFireSE"   ,    10,                                              1.6,    sym_y(0.9 - ROBOT_CENTER_X - FIELD_FIRE_DIST), DIRECTION_FORWARD, PushFieldFire , (-math.pi / 2.0,), False,    True))
        gm.add(goalmanager.Goal("FieldFireE"    ,    10,           1.1 - ROBOT_CENTER_X - FIELD_FIRE_DIST,                                       sym_y(0.4), DIRECTION_FORWARD, PullFieldFire ,            (0.0,), False,    True))
        gm.add(goalmanager.Goal("FieldFireE"    ,    10,           1.1 + ROBOT_CENTER_X + FIELD_FIRE_DIST,                                       sym_y(0.4), DIRECTION_FORWARD, PushFieldFire ,        (math.pi,), False,    True))
        gm.add(goalmanager.Goal("HuntTheMammoth",    19,    0.3 + ROBOT_GYRATION_RADIUS + FIELD_FIRE_DIST,                                  MAMMOTH_HUNT_Y , DIRECTION_FORWARD, HuntTheMammoth,              None, False,    True))
        gm.add(goalmanager.Goal("PaintFresco"   ,    20,                                      RED_START_X,                                            1.30 , DIRECTION_FORWARD, PaintFresco   ,              None, False,    True))
        # gm.add(NoBotherGoal("DontBotherDoc" ,     1,                                      0.52,                                     0.13 , DIRECTION_FORWARD, DontBotherDoc ,              None, False,    True))
        # gm.add(ProtectionGoal("ProtectOurFires",   99,                                             1.67,                                            0.32 , DIRECTION_FORWARD, ProtectOurFires ,              None, False,  True))
        # gm.add(DisabledGoal    ("DepositFires_Mine", 0,                                               0.0,                                            0.0 , DIRECTION_FORWARD,           None ,              None, True,     True))
        # gm.add(calibration_y1)
        # gm.add(calibration_y2)
        # gm.add(calibration_x1)

        yield AntiBlocking(True)
        yield Trigger(ARM_CLOSE)
        yield CalibratePosition()


    def on_start(self, packet):
        self.yield_at(90000, EndOfMatch())
        logger.log("Starting ...")
        #yield FirstHurryToTheOtherMammoth()

        while True :
            yield ExecuteGoals()
            yield Timer(1000)




class CalibratePosition(statemachine.State):

    def on_enter(self):
        if IS_HOST_DEVICE_PC:
            yield DefinePosition(RED_START_X, RED_START_Y, RED_START_ANGLE)
        else:
            estimated_start_y = FIELD_Y_SIZE / 2.0
            yield DefinePosition(0.3 + ROBOT_CENTER_X, estimated_start_y, math.pi)
            yield MoveLineTo(RED_START_X, estimated_start_y, DIRECTION_BACKWARDS)
            yield RotateTo(-math.pi / 2.0)
            yield SpeedControl(0.2)
            yield MoveLineTo(RED_START_X, 0.0)
            yield DefinePosition(None, ROBOT_CENTER_X, -math.pi / 2.0)
            yield MoveLineTo(RED_START_X, RED_START_Y, DIRECTION_BACKWARDS)
            yield RotateTo(math.pi / 2.0)
            yield SpeedControl()
        yield None




class FirstHurryToTheOtherMammoth(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.ServoControl(*ARM_OPEN))
        move = MoveLineTo(RED_START_X, MAMMOTH_HUNT_Y)
        yield move
        self.send_packet(packets.ServoControl(*ARM_CLOSE))
        if move.exit_reason == TRAJECTORY_DESTINATION_REACHED:
            yield HuntTheMammoth()
        self.robot.goal_manager.update_goal_status("HuntTheMammoth", GOAL_DONE)
        yield None


class DontBotherDoc(statemachine.State):

    def on_enter(self):
        yield Timer(1000)
        yield None


class ProtectOurFires(statemachine.State):

    def on_enter(self):
        yield MoveLineRelative(0.15)
        # yield None

class HuntTheMammoth(statemachine.State):

    def on_enter(self):
        if self.robot.team == TEAM_YELLOW:
            yield RotateTo(-math.pi / 2.0)
        else:
            yield RotateTo(math.pi / 2.0)
        yield Timer(300)
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
        yield MoveLineTo(0.2, goal.y)
        yield Trigger(PAINT_1_FLIP_FLOP_STOP, PAINT_1_FLIP_FLOP_STOP)
        self.exit_reason = GOAL_DONE

        yield CalibrateAxis("y", 1.10, math.pi/2)

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

        yield RecalibrateAfterBorderFire(self.goal)

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

        yield RecalibrateAfterBorderFire(self.goal)

        yield None


class RecalibrateAfterBorderFire(statemachine.State):

    def __init__(self, goal):
        self.parent_goal = goal

    def on_enter(self):
        dx = 0.0
        dy = 0.0
        angle = math.pi
        calib_pos = None
        calib_axis = None

        delta = 0.2

        if self.parent_goal.identifier == "BorderFireW" or self.parent_goal.identifier == "BorderFireE":
            dx = +delta

        elif self.parent_goal.identifier == "BorderFireSW":
            dy = -delta

        elif self.parent_goal.identifier == "BorderFireSE":
            dy = delta

        if self.parent_goal.identifier == "BorderFireW":
            angle = math.pi/2
            calib_pos = 0
            calib_axis = "y"

        if self.parent_goal.identifier == "BorderFireE":
            angle = -math.pi/2
            calib_pos = FIELD_Y_SIZE
            calib_axis = "y"

        if self.parent_goal.identifier == "BorderFireSW" or self.parent_goal.identifier == "BorderFireSE":
            calib_pos = FIELD_X_SIZE
            calib_axis = "x"


        yield LookAt(self.parent_goal.x + dx, self.parent_goal.y + dy)
        yield MoveLineTo(self.parent_goal.x + dx, self.parent_goal.y + dy)

        yield CalibrateAxis(calib_axis, calib_pos, angle)

        yield None




class PushFieldFire(statemachine.State):

    def __init__(self, angle):
        self.angle = angle


    def on_enter(self):
        yield RotateTo(self.angle)
        yield Trigger(ARM_OPEN)
        yield MoveLineRelative(FIELD_FIRE_DIST)
        yield Trigger(ARM_CLOSE)
        self.exit_reason = GOAL_DONE
        yield None




class PullFieldFire(statemachine.State):

    def __init__(self, angle):
        self.angle = angle


    def on_enter(self):
        yield RotateTo(self.angle)
        yield MoveLineRelative(0.10)
        yield Trigger(ARM_OPEN)
        yield MoveLineRelative(0.10, DIRECTION_BACKWARDS)
        yield Trigger(ARM_CLOSE)
        self.exit_reason = GOAL_DONE
        yield None




##################################################
# End of match




class EndOfMatch(statemachine.State):

    def on_enter(self):
        yield StopAll();
