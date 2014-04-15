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

    def on_enter(self):
        statemachine.StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_MAIN)
        statemachine.StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_SECONDARY)


    def on_device_ready(self, packet):
        yield AntiBlocking(True)
        yield CalibratePosition()


    def on_start(self, packet):
        self.yield_at(90000, EndOfMatch())
        logger.log("Starting ...")


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

class TakeFruits(statemachine.State):

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
