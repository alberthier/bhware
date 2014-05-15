# encoding: utf-8

import collections
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
FRUIT_TAKE_DISTANCE = 0.28
TREE_APPROACH = TAKE_FRUITS_TRAVEL_DISTANCE / 2.0
TREE_GO_AWAY = TREE_APPROACH
TREE_E_X = 1.3 - TREE_APPROACH
TREE_SE_X = FIELD_X_SIZE - FRUIT_TAKE_DISTANCE
TREE_SW_X = FIELD_X_SIZE - FRUIT_TAKE_DISTANCE
TREE_W_X = 1.3 + TREE_APPROACH
TREE_E_Y = sym_y(FRUIT_TAKE_DISTANCE)
TREE_SE_Y = 2.3 + TREE_APPROACH
TREE_SW_Y = 0.7 + TREE_APPROACH
TREE_W_Y = FRUIT_TAKE_DISTANCE


class FireHarvestingGoal(mgm.Goal):

    def is_available(self):
        robot = self.goal_manager.event_loop.robot
        return super().is_available() and robot.stored_fires < 2




class FireDepositGoal(mgm.Goal):

    def is_available(self):
        robot = self.goal_manager.event_loop.robot
        return super().is_available() and robot.stored_fires != 0




class FruitHarvestingGoal(mgm.Goal):

    def is_available(self):
        robot = self.goal_manager.event_loop.robot
        return self.identifier not in robot.visited_trees




class FruitDepositGoal(mgm.Goal):

    def is_available(self):
        robot = self.goal_manager.event_loop.robot
        return super().is_available() and robot.fruit_tank_trees != 0




class Main(statemachine.State):

    def on_enter(self):
        statemachine.StateMachine(self.event_loop, "interbot")
        statemachine.StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_MAIN)
        statemachine.StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_SECONDARY)
        statemachine.StateMachine(self.event_loop, "relaytoggler")
        self.robot.stored_fires = 0
        self.robot.fruit_tank_trees = 0
        self.robot.visited_trees = []


    def on_device_ready(self, packet):
        gm = self.robot.goal_manager

        self.start_x = 0.3 + ROBOT_GYRATION_RADIUS + 0.03

        FRUIT_SIDE_X_CENTER = 1.3
        FRUIT_SIDE_X_TOP = FRUIT_SIDE_X_CENTER - TAKE_FRUITS_TRAVEL_DISTANCE / 2
        FRUIT_SIDE_X_BOTTOM = FRUIT_SIDE_X_CENTER + TAKE_FRUITS_TRAVEL_DISTANCE / 2
        FRUIT_SIDE_X = FRUIT_SIDE_X_CENTER + TAKE_FRUITS_TRAVEL_DISTANCE / 2
        FRUIT_SIDE_Y = FRUIT_TAKE_DISTANCE
        FRUIT_BOTTOM_X = FIELD_X_SIZE - FRUIT_TAKE_DISTANCE
        FRUIT_BOTTOM_Y_CENTER = 0.7
        # FRUIT_BOTTOM_Y = 1.03

        TAKE_FRUITS_X_DELTA = TAKE_FRUITS_TRAVEL_DISTANCE/2

        MY_TORCH_Y = 0.90


        MINE_TREE_ANGLE          =       -math.pi
        THEIRS_TREE_ANGLE        =            0.0
        MINE_TREE_CENTER_ANGLE   = -math.pi / 2.0
        THEIRS_TREE_CENTER_ANGLE = -math.pi / 2.0
        if self.robot.team == TEAM_YELLOW :
            FRUIT_SIDE_X_BOTTOM, FRUIT_SIDE_X_TOP = FRUIT_SIDE_X_TOP, FRUIT_SIDE_X_BOTTOM
            TAKE_FRUITS_X_DELTA *= -1
            MINE_TREE_ANGLE, THEIRS_TREE_ANGLE = THEIRS_TREE_ANGLE, MINE_TREE_ANGLE
            THEIRS_TREE_CENTER_ANGLE *= -1
            MINE_TREE_CENTER_ANGLE   *= -1


        #                      |        ID           |Weight|     X       |          Y         | Direction          |    State      | Ctor parameters|Shared|Navigate|
        gm.add(
            mgm.Goal           ("HuntTheMammoth"     ,    10, self.start_x,                0.75, DIRECTION_FORWARD  , HuntTheMammoth ,            None, False,    True),

            FireHarvestingGoal ("TakeTorch_Mine"     ,    10,          1.1,          MY_TORCH_Y, DIRECTION_FORWARD  , TakeTorch      ,         (True,), False,    True),
            FireHarvestingGoal ("TakeTorch_Theirs"   ,     1,          1.1,   sym_y(MY_TORCH_Y), DIRECTION_FORWARD  , TakeTorch      ,        (False,), False,    True),

            FireDepositGoal    ("DepositFires_Mine"  ,    10,         1.68,                0.32, DIRECTION_FORWARD  , EmptyFireTank  ,            None, False,    True),
            FireDepositGoal    ("DepositFires_Center",     1,         1.05,                1.50, DIRECTION_FORWARD  , EmptyFireTank  ,            None, False,    True),
            FireDepositGoal    ("DepositFires_Theirs",     3,         1.68,                2.66, DIRECTION_FORWARD  , EmptyFireTank  ,            None, False,    True),

#            FruitHarvestingGoal("FruitTreeE"         ,     3,     TREE_E_X,            TREE_E_Y, DIRECTION_FORWARD  , SuperTakeFruits,            None, False,    True),
#            FruitHarvestingGoal("FruitTreeSE"        ,     3,    TREE_SE_X,           TREE_SE_Y, DIRECTION_FORWARD  , SuperTakeFruits,            None, False,    True),
#            FruitHarvestingGoal("FruitTreeSW"        ,     3,    TREE_SW_X,           TREE_SW_Y, DIRECTION_FORWARD  , SuperTakeFruits,            None, False,    True),
#            FruitHarvestingGoal("FruitTreeW"         ,     3,     TREE_W_X,            TREE_W_Y, DIRECTION_FORWARD  , SuperTakeFruits,            None, False,    True),

            FruitDepositGoal   ("DepFruits_Inner"    ,     7,         0.52,                2.10, DIRECTION_BACKWARDS, DumpFruits     ,            None, False,    True),
            FruitDepositGoal   ("DepFruits_Outer"    ,     7,         0.52,                2.37, DIRECTION_BACKWARDS, DumpFruits     ,            None, False,    True),
        )

#        fruit_goals = (
#            mgm.Goal("Fruits_Mine"      ,   7, FRUIT_SIDE_X_BOTTOM ,         FRUIT_SIDE_Y, DIRECTION_FORWARD, TakeFruits     ,                         (MINE_TREE_ANGLE,), False,    True),
#            mgm.Goal("Fruits_MineCenter",   7, FRUIT_BOTTOM_X      ,FRUIT_BOTTOM_Y_CENTER + TAKE_FRUITS_X_DELTA, DIRECTION_FORWARD, TakeFruits     ,   (MINE_TREE_CENTER_ANGLE,), False,    True),
#            mgm.Goal("Fruits_TheirsCenter", 7, FRUIT_BOTTOM_X      ,sym_y(FRUIT_BOTTOM_Y_CENTER) + TAKE_FRUITS_X_DELTA, DIRECTION_FORWARD, TakeFruits     , (THEIRS_TREE_CENTER_ANGLE,), False,    True),
#            mgm.Goal("Fruits_Theirs"   ,    7, FRUIT_SIDE_X_TOP    ,  sym_y(FRUIT_SIDE_Y), DIRECTION_FORWARD, TakeFruits     ,                         (THEIRS_TREE_ANGLE,), False,    True),
#        )
#
#        gm.add(*fruit_goals)


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




class SuperTakeFruits(statemachine.State):

    def on_enter(self):
        goal = self.robot.goal_manager.get_current_goal()
        if goal.identifier == "W":
            angle = -math.pi
        elif goal.identifier == "E":
            angle = 0.0
        else:
            angle = -math.pi / 2.0
        yield RotateTo(angle, DIRECTION_FORWARD, None, False) # Non-virtual rotation. Team color not taken into account
        yield Trigger(FRUITMOTH_HATCH_OPEN)
        yield Trigger(FRUITMOTH_ARM_OPEN)
        yield Trigger(FRUITMOTH_FINGER_OPEN)
        rfd = 0.08 # Retract finger before tree distance
        dfd = 0.08 # Deploy finger after tree distance
        operations = [
                (None         , 1.3 - rfd         , TREE_E_Y          , FRUITMOTH_FINGER_RETRACT), #  0 - Just before East trunk
                (None         , 1.3 + dfd         , TREE_E_Y          , FRUITMOTH_FINGER_OPEN   ), #  1 - Just after East trunk
                ("FruitTreeE" , 1.3 + TREE_GO_AWAY, TREE_E_Y          , None                    ), #  2 - Finished with the East tree
                (None         , TREE_SE_X         , TREE_SE_Y         , None                    ), #  3 - Approach South-East tree
                (None         , TREE_SE_X         , 2.3 + rfd         , FRUITMOTH_FINGER_RETRACT), #  4 - Just before South-East trunk
                (None         , TREE_SE_X         , 2.3 - dfd         , FRUITMOTH_FINGER_OPEN   ), #  5 - Just after South-East trunk
                ("FruitTreeSE", TREE_SE_X         , 2.3 + TREE_GO_AWAY, None                    ), #  6 - Finished with South-East tree
                (None         , TREE_SW_X         , TREE_SW_Y         , None                    ), #  7 - Approach South-West tree
                (None         , TREE_SW_X         , 0.7 + rfd         , FRUITMOTH_FINGER_RETRACT), #  8 - Just before South-West trunk
                (None         , TREE_SW_X         , 0.7 - dfd         , FRUITMOTH_FINGER_OPEN   ), #  9 - Just after South-West trunk
                ("FruitTreeSW", TREE_SW_X         , 0.7 + TREE_GO_AWAY, None                    ), # 10 - Finished with South-West tree
                (None         , TREE_W_X          , TREE_W_Y          , None                    ), # 11 - Approach West tree
                (None         , 1.3 + rfd         , TREE_W_Y          , FRUITMOTH_FINGER_RETRACT), # 12 - Just before West trunk
                (None         , 1.3 - dfd         , TREE_W_Y          , FRUITMOTH_FINGER_OPEN   ), # 13 - Just after West trunk
                ("FruitTreeW" , 1.3 + TREE_GO_AWAY, TREE_W_Y          , None                    ), # 14 - Finished with West tree
                ]
        if goal.identifier == "FruitTreeE":
            start = 0
        elif goal.identifier == "FruitTreeSE":
            start = 3
        elif goal.identifier == "FruitTreeSW":
            start = 7
        else:
            start = 11
        if "FruitTreeW" not in self.robot.visited_trees:
            end = 14
        elif "FruitTreeSW" not in self.robot.visited_trees:
            end = 10
        elif "FruitTreeSE" not in self.robot.visited_trees:
            end = 6
        else:
            end = 2
        yield SuperTakeFruitsMovement(operations[start : end])
        yield Trigger(FRUITMOTH_FINGER_CLOSE, FRUITMOTH_ARM_CLOSE)
        yield Trigger(FRUITMOTH_HATCH_CLOSE)





class SuperTakeFruitsMovement(statemachine.State):

    def __init__(self, operations):
        self.path = []
        self.commands = collections.deque()
        self.done_trees = collections.deque()
        for tree, x, y, cmd in operations:
            self.done_trees.append(tree)
            self.path.append((x, y))
            self.commands.append(cmd)


    def on_enter(self):
        move = MoveCurve(path)
        move.on_waypoint_reached = self.on_waypoint_reached
        yield move
        yield None


    def on_waypoint_reached(self, packet):
        cmd = self.commands.popleft()
        if cmd is not None:
            yield Trigger(cmd)
        tree = self.done_trees.popleft()
        if tree is not None:
            self.robot.visited_trees.append(tree)




class TakeFruits(statemachine.State):
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
        x = self.goal.x
        y = self.goal.y
        if self.robot.team == TEAM_YELLOW:
            y = sym_y(y)
        increment_1 = 0.2
        increment_2 = 0.3
        increment_3 = 0.5
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
        yield commonstates.SpeedControl()

        self.robot.fruit_tank_full = True
        self.exit_reason = GOAL_DONE

        yield None


    def on_waypoint_reached(self, packet):
        if self.retract_finger:
            self.retract_finger = False
            yield Trigger(FRUITMOTH_FINGER_RETRACT)
        else:
            yield Trigger(FRUITMOTH_FINGER_OPEN)


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
        yield Trigger(ARM_1_TAKE_TORCH_FIRE, ARM_2_TAKE_TORCH_FIRE)
        yield Trigger(ELEVATOR_LEVEL_2) # This is absolutely requierd to avoid elevator damages
        yield Trigger(MAMMOTH_NET_THROW)
        yield StopAll()
