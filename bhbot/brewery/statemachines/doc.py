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




class FireHarvestingGoal(goalmanager.Goal):

    def is_available(self):
        robot = self.goal_manager.event_loop.robot
        return super().is_available() and robot.stored_fires < 2




class FireDepositGoal(goalmanager.Goal):

    def is_available(self):
        robot = self.goal_manager.event_loop.robot
        return super().is_available() and robot.stored_fires != 0




class FruitDepositGoal(goalmanager.Goal):

    def is_available(self):
        robot = self.goal_manager.event_loop.robot
        return super().is_available() and robot.fruit_tank_full




class Main(statemachine.State):

    def on_enter(self):
        statemachine.StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_MAIN)
        statemachine.StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_SECONDARY)
        statemachine.StateMachine(self.event_loop, "relaytoggler")


    def on_device_ready(self, packet):
        gm = self.robot.goal_manager

        self.start_x = 0.3 + ROBOT_GYRATION_RADIUS + 0.03

        #                        |       ID         |Weight|                        X                      |                  Y                     |     Direction    | State         | Ctor parameters             |Shared|Navigate|
        gm.add(goalmanager.Goal  ("HuntTheMammoth"  ,  10,                                self.start_x ,                                        0.75, DIRECTION_FORWARD, HuntTheMammoth,                         None, False,    True))
        gm.add(FireHarvestingGoal("TakeMyTorch"     ,  10,                                         1.1 ,                                        0.90, DIRECTION_FORWARD, TakeTorch     ,                      (True,), False,    True))
        gm.add(FireHarvestingGoal("TakeTheirTorch"  ,   1,                                         1.1 ,                         FIELD_Y_SIZE - 0.90, DIRECTION_FORWARD, TakeTorch     ,                     (False,), False,    True))
#        gm.add(FireDepositGoal   ("DepositFires"    ,  10, FIELD_X_SIZE - 0.25 - ROBOT_GYRATION_RADIUS ,                0.25 + ROBOT_GYRATION_RADIUS, DIRECTION_FORWARD, EmptyFireTank ,                         None, False,    True))
#        gm.add(FireDepositGoal   ("DepositFires"    ,   1, FIELD_X_SIZE - 0.25 - ROBOT_GYRATION_RADIUS , FIELD_Y_SIZE - 0.25 + ROBOT_GYRATION_RADIUS, DIRECTION_FORWARD, EmptyFireTank ,                         None, False,    True))
#        gm.add(FireDepositGoal   ("DepositFires"    ,   3,                                        1.05 ,                                        1.50, DIRECTION_FORWARD, EmptyFireTank ,                         None, False,    True))
#        gm.add(goalmanager.Goal  ("TakeFruitE"      ,   7,                                        1.05 ,                                        1.50, DIRECTION_FORWARD, TakeFruit     ,                         None, False,    True))
#        gm.add(goalmanager.Goal  ("TakeFruitSE"     ,   7,                                        1.05 ,                                        1.50, DIRECTION_FORWARD, TakeFruit     ,                         None, False,    True))
#        gm.add(goalmanager.Goal  ("TakeFruitSW"     ,   7,                                        1.05 ,                                        1.50, DIRECTION_FORWARD, TakeFruit     ,                         None, False,    True))
#        gm.add(goalmanager.Goal  ("TakeFruitW"      ,   7,                                        1.05 ,                                        1.50, DIRECTION_FORWARD, TakeFruit     ,                         None, False,    True))
#        gm.add(FruitDepositGoal  ("DepositFruits"   ,   7,                                        1.05 ,                                        1.50, DIRECTION_FORWARD, DepositFruits ,                         None, False,    True))

        yield AntiBlocking(True)
        yield CalibratePosition()


    def on_start(self, packet):
        self.yield_at(90000, EndOfMatch())
        logger.log("Starting ...")
        yield Timer(1000)
        yield MoveLineTo(self.start_x, RED_START_Y)
        yield ExecuteGoals()




class CalibratePosition(statemachine.State):

    def on_enter(self):
        if IS_HOST_DEVICE_PC:
            yield DefinePosition(RED_START_X, RED_START_Y, RED_START_ANGLE)
        else:
            estimated_start_y = FIELD_Y_SIZE / 2.0
            yield DefinePosition(ROBOT_CENTER_X, estimated_start_y, 0.0)
            yield MoveLineTo(0.5, estimated_start_y)
            yield Rotate(math.pi / 2.0)
            yield SpeedControl(0.2)
            yield MoveLineTo(0.5, 0.0, DIRECTION_BACKWARDS)
            yield DefinePosition(None, ROBOT_CENTER_X, math.pi / 2.0)
            yield SpeedControl()
            yield MoveLineTo(0.5, RED_START_Y)
            yield Rotate(0.0)
            yield MoveLineTo(RED_START_X, RED_START_Y)
        yield None




class HuntTheMammoth(statemachine.State):

    def on_enter(self):
        yield RotateTo(0.0)
        yield Trigger(GUN_FIRE)
        yield None




class TakeTorch(statemachine.State):

    def __init__(self, my_side):
        self.my_side = my_side


    def on_enter(self):
        flip = not self.my_side
        yield Trigger(ELEVATOR_UP)
        for i in range(3):
            yield Trigger(ARM_1_TAKE_TORCH_FIRE, ARM_2_TAKE_TORCH_FIRE)
            yield Trigger(PUMP_ON, makeServoMoveCommand(ELEVATOR, 90 - 30 * i))
            yield Trigger(makeServoMoveCommand(ELEVATOR, 100))
            if flip:
                yield Trigger(FIRE_FLIPPER_OPEN, ARM_1_FLIP_FIRE, ARM_2_FLIP_FIRE)
            else:
                yield Trigger(ARM_1_STORE_FIRE, ARM_2_STORE_FIRE)
            flip = not flip
            yield Trigger(PUMP_OFF)
        yield None




class TakeFruit(statemachine.State):

    def __init__(self, angle):
        self.angle = angle


    def on_enter(self):
        cos_angle = math.cos(self.angle)
        if abs(cos_angle) > 0.5:
            x_orientation = 1.0 if math.cos(self.angle) > 0 else -1.0
            y_orientation = 0.0
        else:
            x_orientation = 0.0
            y_orientation = -1.0

        yield RotateTo(self.angle)
        # slow down
        yield commonstates.SpeedControl(0.2)
        # Ouvrir la trappe
        Trigger(FRUITMOTH_HATCH_OPEN)
        # Sortir le développeur et le doigt
        Trigger(FRUITMOTH_ARM_OPEN, FRUITMOTH_FINGER_OPEN)

        self.retract_finger = True
        x = self.robot.goal_manager.get_current_goal().x
        y = self.robot.goal_manager.get_current_goal().y
        increment_1 = 0.1
        increment_2 = 0.2
        increment_3 = 0.3
        move = MoveLine([Pose(x + x_orientation * increment_1, y + y_orientation * increment_1),
                         Pose(x + x_orientation * increment_2, y + y_orientation * increment_2),
                         Pose(x + x_orientation * increment_3, y + y_orientation * increment_3)])
        move.on_waypoint_reached = self.on_waypoint_reached
        yield move

        # steps will be handled in on_waypoint_reached
        yield Trigger(FRUITMOTH_FINGER_CLOSE)
        # Fermer le bras
        yield Trigger(FRUITMOTH_ARM_CLOSE)
        #fermer la trappe
        yield Trigget(FRUITMOTH_HATCH_CLOSE)
        #standard speed
        yield commonstates.SpeedControl()
        yield None


    def on_waypoint_reached(self, packet):
        if self.retract_finger:
            self.retract_finger = False
            Trigger(FRUITMOTH_FINGER_RETRACT)
        else:
            Trigger(FRUITMOTH_FINGER_OPEN)



class DumpFruits(statemachine.State):

    def on_enter(self):

        #ouvrir la trappe
        yield commonstates.ServoControl(FRUITMOTH_HATCH_OPEN)
        #sortir le développeur
        yield commonstates.ServoControl(FRUITMOTH_ARM_OPEN)

        #fermer la trappe
        yield commonstates.ServoControl(FRUITMOTH_HATCH_CLOSE)

        #incliner le bac
        yield commonstates.ServoControl(FRUITMOTH_TUB_OPEN)

        #rentrer le bac
        yield commonstates.ServoControl(FRUITMOTH_TUB_CLOSE)

        #ouvrir la trappe
        yield commonstates.ServoControl(FRUITMOTH_HATCH_OPEN)

        #rentrer le développeur
        yield commonstates.ServoControl(FRUITMOTH_ARM_CLOSE)

        #fermer la trappe
        yield commonstates.ServoControl(FRUITMOTH_HATCH_CLOSE)





##################################################
# End of match - Baloon




class EndOfMatch(statemachine.State):

    def on_enter(self):
        yield StopAll()
