# encoding: utf-8

import math

import statemachine
import packets
import position
import logger
import commonstates
import goalmanager as mgm

from definitions import *
from commonstates import *
from position import *
from tools import *

# global constants

TAKE_FRUITS_TRAVEL_DISTANCE = 0.5


class FireHarvestingGoal(mgm.Goal):

    def is_available(self):
        robot = self.goal_manager.event_loop.robot
        return super().is_available() and robot.stored_fires < 2




class FireDepositGoal(mgm.Goal):

    def is_available(self):
        robot = self.goal_manager.event_loop.robot
        return super().is_available() and robot.stored_fires != 0




class FruitDepositGoal(mgm.Goal):

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

        FRUIT_TAKE_DISTANCE = 0.28

        FRUIT_SIDE_X_CENTER = 1.3
        FRUIT_SIDE_X_TOP = FRUIT_SIDE_X_CENTER - TAKE_FRUITS_TRAVEL_DISTANCE/2
        FRUIT_SIDE_X_BOTTOM = FRUIT_SIDE_X_CENTER + TAKE_FRUITS_TRAVEL_DISTANCE/2
        FRUIT_SIDE_X = FRUIT_SIDE_X_CENTER + TAKE_FRUITS_TRAVEL_DISTANCE/2
        FRUIT_SIDE_Y = FRUIT_TAKE_DISTANCE
        FRUIT_BOTTOM_X = FIELD_X_SIZE - FRUIT_TAKE_DISTANCE
        FRUIT_BOTTOM_Y_CENTER = 0.7
        # FRUIT_BOTTOM_Y = 1.03

        TAKE_FRUITS_X_DELTA = TAKE_FRUITS_TRAVEL_DISTANCE/2

        MY_TORCH_Y = 0.90

        if self.robot.team == TEAM_YELLOW :
            FRUIT_SIDE_X_BOTTOM, FRUIT_SIDE_X_TOP = FRUIT_SIDE_X_TOP, FRUIT_SIDE_X_BOTTOM
            TAKE_FRUITS_X_DELTA*=-1

        #                        |       ID       |Weight| X          | Y                  | Direction         | State      | Ctor parameters|Shared|Navigate|
        gm.add(
            mgm.Goal          ("HuntTheMammoth"  ,  10,  self.start_x,               0.75, DIRECTION_FORWARD, HuntTheMammoth,           None, False,    True),

            FireHarvestingGoal("TakeMyTorch"     ,  10,           1.1,         MY_TORCH_Y, DIRECTION_FORWARD,  TakeTorch     ,       (True,), False,    True),
            FireHarvestingGoal("TakeTheirTorch"  ,   1,           1.1,   sym_y(MY_TORCH_Y), DIRECTION_FORWARD, TakeTorch     ,      (False,), False,    True),

            FireDepositGoal   ("Deposit_Mine"    ,  10,          1.68,               0.32, DIRECTION_FORWARD, EmptyFireTank ,           None, False,    True),
            FireDepositGoal   ("Deposit_Center"  ,   1,          1.05,               1.50, DIRECTION_FORWARD, EmptyFireTank ,           None, False,    True),
            FireDepositGoal   ("Deposit_Theirs"  ,   3,          1.68,               2.66, DIRECTION_FORWARD, EmptyFireTank ,           None, False,    True),

            FruitDepositGoal  ("DepFruits_Inner" ,   7,          0.52,               2.10, DIRECTION_BACKWARDS, DumpFruits ,            None, False,    True),
            FruitDepositGoal  ("DepFruits_Outer" ,   7,          0.52,               2.37, DIRECTION_BACKWARDS, DumpFruits ,            None, False,    True),
        )

        fruit_goals = (
            mgm.Goal("Fruits_Mine"      ,   7, FRUIT_SIDE_X_BOTTOM ,         FRUIT_SIDE_Y, DIRECTION_FORWARD, TakeFruits     ,                         None, False,    True),
            mgm.Goal("Fruits_MineCenter",   7, FRUIT_BOTTOM_X      ,FRUIT_BOTTOM_Y_CENTER + TAKE_FRUITS_X_DELTA, DIRECTION_FORWARD, TakeFruits     ,None, False,    True),
            mgm.Goal("Fruits_TheirsCenter", 7, FRUIT_BOTTOM_X      ,sym_y(FRUIT_BOTTOM_Y_CENTER) + TAKE_FRUITS_X_DELTA, DIRECTION_FORWARD, TakeFruits     , None, False,    True),
            mgm.Goal("Fruits_Theirs"   ,    7, FRUIT_SIDE_X_TOP    ,  sym_y(FRUIT_SIDE_Y), DIRECTION_FORWARD, TakeFruits     ,                         None, False,    True),
        )

        gm.add(*fruit_goals)


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
        self.exit_reason = GOAL_DONE
        yield None




class TakeTorch(statemachine.State):

    def __init__(self, my_side):
        self.my_side = my_side


    def on_enter(self):
        flip = self.my_side
        yield Trigger(ELEVATOR_UP, FIRE_FLIPPER_OPEN)
        elevator_take_levels = [ELEVATOR_LEVEL_3, ELEVATOR_LEVEL_2, ELEVATOR_LEVEL_1]
        elevator_store_levels = [ELEVATOR_LEVEL_1, ELEVATOR_LEVEL_2, ELEVATOR_LEVEL_3, ELEVATOR_UP]
        for i in range(3):
            yield Trigger(ARM_1_TAKE_TORCH_FIRE, ARM_2_TAKE_TORCH_FIRE)
            yield Trigger(PUMP_ON, elevator_take_levels[i])
            yield Timer(300)
            yield Trigger(ELEVATOR_UP)
            yield from self.arm_speed(200)
            if flip:
                # On retourne le feu
                yield Trigger(ARM_1_FLIP_FIRE, ARM_2_FLIP_FIRE)
                yield Trigger(PUMP_OFF)
                yield Timer(300)
            else:
                # Sinon, on le stocke
                yield Trigger(ARM_1_STORE_FIRE, ARM_2_STORE_FIRE)
                yield Trigger(elevator_store_levels[self.robot.stored_fires])
                yield Trigger(PUMP_OFF)
                yield Timer(300)
                yield Trigger(ELEVATOR_UP)
                self.robot.stored_fires += 1
                yield from self.arm_speed(1023)
            flip = not flip
        yield Trigger(FIRE_FLIPPER_CLOSE)
        self.exit_reason = GOAL_DONE
        yield None


    def arm_speed(self, speed):
        yield Trigger(makeServoSetupCommand(ARM_1, speed), makeServoSetupCommand(ARM_2, speed))




class TakeFruits(statemachine.State):

    def __init__(self, angle=0.0):
        #self.angle = self.robot.pose.angle
        pass


    def on_enter(self):
        self.angle = self.robot.pose.angle
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
        yield Trigger(FRUITMOTH_HATCH_CLOSE)
        #standard speed
        # yield commonstates.SpeedControl()

        self.robot.fruit_tank_full = True
        self.exit_reason = GOAL_DONE


        yield None


    def on_waypoint_reached(self, packet):
        if self.retract_finger:
            self.retract_finger = False
            Trigger(FRUITMOTH_FINGER_RETRACT)
        else:
            Trigger(FRUITMOTH_FINGER_OPEN)


class DumpFruits(statemachine.State):

    def on_enter(self):
        yield RotateTo(0.0)

        #ouvrir la trappe
        yield Trigger(FRUITMOTH_HATCH_OPEN)
        #sortir le développeur
        yield Trigger(FRUITMOTH_ARM_OPEN)

        #fermer la trappe
        yield Trigger(FRUITMOTH_HATCH_CLOSE)
        self.robot.fruit_tank_full=False
        self.exit_reason=GOAL_DONE

        #incliner le bac
        yield Trigger(FRUITMOTH_TUB_OPEN)

        #rentrer le bac
        yield commonstates.ServoControl(FRUITMOTH_TUB_CLOSE)

        self.exit_reason = GOAL_DONE

        yield None

class EmptyFireTank(statemachine.State):

    def on_enter(self):
        if self.goal.identifier == 'Deposit_Mine' :
            yield RotateTo(- math.pi / 4.0)
        elif self.goal.identifier == 'Deposit_Theirs' :
            yield RotateTo( math.pi / 4.0)

        self.exit_reason = GOAL_DONE
        self.robot.stored_fires = 0
        yield None


##################################################
# End of match - Baloon




class EndOfMatch(statemachine.State):

    def on_enter(self):
        yield StopAll()
