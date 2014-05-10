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
        gm.add(FireDepositGoal   ("DepositFires"    ,  10, FIELD_X_SIZE - 0.25 - ROBOT_GYRATION_RADIUS ,                0.25 + ROBOT_GYRATION_RADIUS, DIRECTION_FORWARD, EmptyFireTank ,                         None, False,    True))
        gm.add(FireDepositGoal   ("DepositFires"    ,   1, FIELD_X_SIZE - 0.25 - ROBOT_GYRATION_RADIUS , FIELD_Y_SIZE - 0.25 + ROBOT_GYRATION_RADIUS, DIRECTION_FORWARD, EmptyFireTank ,                         None, False,    True))
        gm.add(FireDepositGoal   ("DepositFires"    ,   3,                                        1.05 ,                                        1.50, DIRECTION_FORWARD, EmptyFireTank ,                         None, False,    True))
        gm.add(goalmanager.Goal  ("TakeFruitE"      ,   7,                                        1.05 ,                                        1.50, DIRECTION_FORWARD, TakeFruit     ,                         None, False,    True))
        gm.add(goalmanager.Goal  ("TakeFruitSE"     ,   7,                                        1.05 ,                                        1.50, DIRECTION_FORWARD, TakeFruit     ,                         None, False,    True))
        gm.add(goalmanager.Goal  ("TakeFruitSW"     ,   7,                                        1.05 ,                                        1.50, DIRECTION_FORWARD, TakeFruit     ,                         None, False,    True))
        gm.add(goalmanager.Goal  ("TakeFruitW"      ,   7,                                        1.05 ,                                        1.50, DIRECTION_FORWARD, TakeFruit     ,                         None, False,    True))
        gm.add(FruitDepositGoal  ("DepositFruits"   ,   7,                                        1.05 ,                                        1.50, DIRECTION_FORWARD, DepositFruits ,                         None, False,    True))


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
        yield None




class DropTorch(statemachine.State):

    def on_enter(self):
        #sortir guide torche
        #mettre bras 1 et 2 en position au dessus de la torche
        yield commonstates.ServoControl(TORCH_GUIDE_OPEN, ARM_1_CLOSE, ARM_2_CLOSE)
        #descendre bras
        yield commonstates.ServoControl(ELEVATOR_DOWN)
        #allumer pompe (paquet à créer)
        #yield commonstates.ServoControl(ELEVATOR_DOWN)
        #remonter bras
        yield commonstates.ServoControl(ELEVATOR_UP)
        #bras en position avant
        yield commonstates.ServoControl(ARM_1_OPEN, ARM_2_OPEN)
        #descendre bras
        yield commonstates.ServoControl(ELEVATOR_DOWN)
        #eteindre pompe (paquet à créer)
        #yield commonstates.ServoControl(ELEVATOR_DOWN)
        #remonter bras
        yield commonstates.ServoControl(ELEVATOR_UP)

class TakeFruit(statemachine.State):

    def on_enter(self):
        #slow down
        yield commonstates.SpeedControl(0.2)

        #ouvrir la trappe
        yield commonstates.ServoControl(FRUITMOTH_HATCH_OPEN)
        #sortir le développeur
        yield commonstates.ServoControl(FRUITMOTH_ARM_OPEN)
        #ouvrir le doigt
        yield commonstates.ServoControl(FRUITMOTH_FINGER_OPEN)
        #avancer avec le robot
        self.robot.move.forward(10)
        #rentrer le doigt
        yield commonstates.ServoControl(FRUITMOTH_FINGER_CLOSE)
        #avancer avec le robot
        self.robot.move.forward(10)
        yield commonstates.ServoControl(FRUITMOTH_FINGER_OPEN)
        #sortir le doigt
        self.robot.move.forward(10)
        yield commonstates.ServoControl(FRUITMOTH_FINGER_CLOSE)

        #puis tout rentrer à l'inverse du début

        #fermer le bras
        yield commonstates.ServoControl(FRUITMOTH_ARM_CLOSE)

        #fermer la trappe
        yield commonstates.ServoControl(FRUITMOTH_HATCH_CLOSE)

        #standard speed
        yield commonstates.SpeedControl()

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
