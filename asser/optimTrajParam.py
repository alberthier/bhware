#!/usr/bin/python
# -*- coding: utf-8 -*-


import subprocess
import sys
import platform
import time
from matplotlib.pyplot import *
import numpy
import math
import re
from scipy.optimize.optimize import fmin
from scipy.optimize import fmin_slsqp
from config import *

class commandMsg:
    def __init__(self, header):
        self.cmd = header
        self.msgPose = ""
        self.nbPose = 0

    def addPose(self, strPose):
        self.msgPose = self.msgPose + " " + strPose
        self.nbPose = self.nbPose + 1

    def cmdMsgGeneration(self):
        return self.cmd + " " + str(self.nbPose) + self.msgPose + '\n'

def lineForm(l_char):
    l_line = []
    line = ''
    for char in l_char:
        if char != '\n':
            line = line + char
        else:
            l_line.append(line)
            line = ''
    return l_line

def stdoutParser(l_line):
    d_traj = {'message': []}
    for line in l_line:
        ignored=True
        l_linedata = line.split(" ")
        if len(l_linedata) > 0:
            #if l_linedata[0] == "log_data:":
            #    print(line)
            if l_linedata[0].count('log_', 0, 4) == 1:
                l_linedata[0] = l_linedata[0].replace("log_", "")
                l_linedata[0] = l_linedata[0].replace(":", "")
                if d_traj.keys().count(l_linedata[0]) == 0:
                    d_traj[l_linedata[0]] = []
                d_traj[l_linedata[0]].append(float(l_linedata[1]))
                ignored=False
                
            if l_linedata[0].count('msg_', 0, 4) == 1 :
                d_traj['message'].append(line[4:])
                
            if l_linedata[0].count('logStr_', 0, 7) == 1:
                l_linedata[0] = l_linedata[0].replace("logStr_", "")
                l_linedata[0] = l_linedata[0].replace(":", "")
                if d_traj.keys().count(l_linedata[0]) == 0:
                    d_traj[l_linedata[0]] = []
                if len(l_linedata) > 2:
                    textMsg = ""
                    for text in l_linedata[1:]:
                        textMsg = textMsg + text + " "
                else:
                    textMsg = l_linedata[1]
                d_traj[l_linedata[0]].append(textMsg)
                ignored=False
            if ignored and line.startswith("LOG ") : print line[4:]
    file("log.txt","w").write(str(d_traj))
    return d_traj


def MSG_init_pose(x=0.0, y=0.0, angle=0.0) :
    #generation d'un message d'init de pose
    poseInit = commandMsg("INIT_POSE_ROBOT 0 0 " + str(angle))
    poseInit.addPose(str(x) + " " + str(y))
    return poseInit.cmdMsgGeneration()

def send_init_pose(process, x=0.0, y=0.0, angle=0.0) :
    process.stdin.write(MSG_init_pose(x, y, angle))

def MSG_config_PI(Kp, Ki) :
    #generation d'un message de configuration des gains de l'asser de vitesse PI
    parametersPI = commandMsg("PARAMETERS_PI")
    parametersPI.addPose("KP" + " " + str(Kp))
    parametersPI.addPose("KI" + " " + str(Ki))
    return parametersPI.cmdMsgGeneration()

def send_config_PI(process, Kp, Ki) :
    process.stdin.write(MSG_config_PI(Kp, Ki))

def MSG_config_AsserTrajectoire(K1, K2, K3) :
    #generation d'un message de configuration des gains de l'asser de déplacement longitudinal
    parametersK = commandMsg("PARAMETERS_GAIN_K")
    parametersK.addPose("GAIN_K1" + " " + str(K1))
    parametersK.addPose("GAIN_K2" + " " + str(K2))
    parametersK.addPose("GAIN_K3" + " " + str(K3))
    return parametersK.cmdMsgGeneration()

def send_config_AsserTrajectoire(process, K1, K2, K3) :
    process.stdin.write(MSG_config_AsserTrajectoire(K1, K2, K3))

def MSG_config_AsserRotation(R1) :
    #generation d'un message de configuration des gains de l'asser de rotation
    parametersR = commandMsg("PARAMETERS_GAIN_ROT")
    parametersR.addPose("GAIN_R1" + " " + str(R1))
    return parametersR.cmdMsgGeneration()

def send_config_AsserRotation(process, R1) :
    process.stdin.write(MSG_config_AsserRotation(R1))

def MSG_config_profilVitesse(Amax, Dmax) :
    #generation d'un message de configuration des parametres du profil de vitesse
    parametersT = commandMsg("PARAMETERS_TIME")
    parametersT.addPose("A_MAX" + " " + str(Amax))
    parametersT.addPose("D_MAX" + " " + str(Dmax))
    return parametersT.cmdMsgGeneration()

def MSG_configGenerale(Ratio_Acc, Ratio_Decc) :
    #generation d'un message de configuration des parametres haut niveau du profil de vitesse
    parametersT = commandMsg("CONFIG_ASSER")
    parametersT.addPose("RATIO_ACC" + " " + str(Ratio_Acc))
    parametersT.addPose("RATIO_DECC" + " " + str(Ratio_Decc))
    return parametersT.cmdMsgGeneration()

def send_config_profilVitesse(process, Amax, Dmax) :
    process.stdin.write(MSG_config_profilVitesse(Amax, Dmax))

def send_configGenerale(process, Ratio_Acc, Ratio_Decc) :
    process.stdin.write(MSG_configGenerale(Ratio_Acc, Ratio_Decc))

def MSG_config_modeleMoteurCC(masse,
                                rayon_roue,
                                frottement_fluide,
                                force_resistante,
                                resistance_induit,
                                inductance_induit,
                                constance_couple,
                                constante_vitesse,
                                rapport_reduction) :

    #generation d'un message d'init des parametres du modele des moteurs
    parametersMotor = commandMsg("PARAMETERS_MOTOR")
    parametersMotor.addPose("MASSE" + " " + str(masse))
    parametersMotor.addPose("RAYON_ROUE" + " " + str(rayon_roue))
    parametersMotor.addPose("FROTTEMENT_FLUIDE" + " " + str(frottement_fluide))
    parametersMotor.addPose("FORCE_RESISTANTE" + " " + str(force_resistante))
    parametersMotor.addPose("RESISTANCE_INDUIT" + " " + str(resistance_induit))
    parametersMotor.addPose("INDUCTANCE_INDUIT" + " " + str(inductance_induit))
    parametersMotor.addPose("CONSTANTE_COUPLE" + " " + str(constance_couple))
    parametersMotor.addPose("CONSTANTE_VITESSE" + " " + str(constante_vitesse))
    parametersMotor.addPose("RAPPORT_REDUCTION" + " " + str(rapport_reduction))
    return parametersMotor.cmdMsgGeneration()

def send_config_modeleMoteurCC(process,
                                masse,
                                rayon_roue,
                                frottement_fluide,
                                force_resistante,
                                resistance_induit,
                                inductance_induit,
                                constance_couple,
                                constante_vitesse,
                                rapport_reduction) :
    process.stdin.write(MSG_config_modeleMoteurCC(masse,
                                                    rayon_roue,
                                                    frottement_fluide,
                                                    force_resistante,
                                                    resistance_induit,
                                                    inductance_induit,
                                                    constance_couple,
                                                    constante_vitesse,
                                                    rapport_reduction))

def send_config_simulator(simulator_process, d_cfgTraj) :
    # envoie au simulateur de la configuration des gains de l'asser de vitesse PI
    send_config_PI(simulator_process, d_cfgTraj['Kp'], d_cfgTraj['Ki'])

    # envoie au simulateur de la configuration des gains de l'asser de déplacement longitudinal (l'asser haut niveau)
    send_config_AsserTrajectoire(simulator_process, d_cfgTraj['K1'], d_cfgTraj['K2'], d_cfgTraj['K3'])

    # envoie au simulateur de la configuration des gains de l'asser de rotation
    send_config_AsserRotation(simulator_process, d_cfgTraj['R1'])

    # envoie au simulateur de la configuration des parametres du profil de vitesse
    send_config_profilVitesse(simulator_process, d_cfgTraj['Amax'], d_cfgTraj['Dmax'])

    # envoie au simulateur de la configuration des parametres haut niveau du profil de vitesse
    send_configGenerale(simulator_process, d_cfgTraj['RatioAcc'], d_cfgTraj['RatioDecc'])

    # envoie au simulateur des parametres du modele des moteurs à courant continu de déplacement
    send_config_modeleMoteurCC(simulator_process,
                                d_cfgTraj['Masse'],
                                d_cfgTraj['Rayon_roue'],
                                d_cfgTraj['Frottement_fluide'],
                                d_cfgTraj['Force_resistante'],
                                d_cfgTraj['Resistance_induit'],
                                d_cfgTraj['Inductance_induit'],
                                d_cfgTraj['Constance_couple'],
                                d_cfgTraj['Constante_vitesse'],
                                d_cfgTraj['Rapport_reduction'])


def trajFunction(d_cfgTraj):

    #lancement du simulateur de deplacement
    print("Lancement du simulateur")
    
    if (platform.system() == 'Linux') :
        shellCommand = './simulator_trajAsser'
    if (platform.system() == 'Windows') :
        shellCommand = 'simulator_trajAsser.exe'
    simulator_process = subprocess.Popen(shellCommand, shell=True, stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.PIPE)

    # envoie de la configuration du simulateur
    send_config_simulator(simulator_process, d_cfgTraj)
    
    #############
    send_init_pose(simulator_process, x=0.2, y=0.2, angle=0.0) #4.71
 
    ###############
    #test ROTATE
    #~ send_init_pose(simulator_process, x=0.2, y=0.2, angle=math.pi/2.0) #4.71
    #~ deplacement = commandMsg("MSG_ROTATE 0 -1.57")
    #~ simulator_process.stdin.write(deplacement.cmdMsgGeneration())

    #test MOVE_CURVE
    #~ send_init_pose(simulator_process, x=0.2, y=1.0, angle=0.0)
    #~ deplacement = commandMsg("MSG_MOVE_CURVE 1 1 0.0")
    #~ deplacement.addPose("0.9 1.0")
    #deplacement.addPose("1.0 1.0")
    #~ deplacement.addPose("1.05 0.85")
    #~ deplacement.addPose("1.2 1.0")
    #~ deplacement.addPose("1.35 0.85")
    
    #~ deplacement.addPose("0.5 1.0")
    #~ deplacement.addPose("1.0 1.0")
    #~ deplacement.addPose("1.5 1.0")
    #~ deplacement.addPose("2.0 1.0")
    
    #~ simulator_process.stdin.write(deplacement.cmdMsgGeneration())
    #~ print(MSG_init_pose(0.2, 0.2, 0.0))
    # INIT_POSE_ROBOT 0 0 0.0 1 0.2 0.2
    #~ print(deplacement.cmdMsgGeneration())
    # MSG_MOVE_CURVE 1 1 0.0 4 0.9 1.0 1.05 0.85 1.2 1.0 1.35 0.85
    
    #test MOVE_LINE
    #~ deplacement = commandMsg("MSG_MOVE_LINE 1")
    #~ deplacement.addPose("1.2 0.2")
    #~ #deplacement.addPose("1.3 0.2")
    #~ simulator_process.stdin.write(deplacement.cmdMsgGeneration())
    #~ print(deplacement.cmdMsgGeneration())
    # MSG_MOVE_LINE 1 1 1.0 0.2
    
    #test MOVE_ARC
    deplacement = commandMsg("MSG_MOVE_ARC 1 0.2 1.0 0.8")
    deplacement.addPose(str(- (2.0 * math.pi) / 8.0))
    deplacement.addPose(str(- (0.0 * math.pi) / 8.0))
    
    #~ deplacement.addPose(str(- (6.0 * math.pi) / 8.0))
    #~ deplacement.addPose(str(- (8.0 * math.pi) / 8.0))
    
    simulator_process.stdin.write(deplacement.cmdMsgGeneration())
    
    
    #transmission de la commande de d'arret du simulateur
    (stdoutdata, stderrdata) = simulator_process.communicate("QUIT\n")

    print("Simulation terminee.")
    #traitement des donnees de stdout
    lines = lineForm(stdoutdata)
    d_traj = stdoutParser(lines)

    return d_traj

def trajTest(d_cfgTraj):

    #lancement du simulateur de deplacement
    print("Lancement du simulateur")
    simulator_process = subprocess.Popen('simulator_trajAsser.exe', shell=True, stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.PIPE)

    # envoie de la configuration du simulateur
    send_config_simulator(simulator_process, d_cfgTraj)

    # envoie au simulateur de l'init de pose
    # send_init_pose(simulator_process, x=0.31, y=2.823, angle=1.5708) #1.5708


    #~ send_init_pose(simulator_process, x=0.0, y=0.0, angle=3.1415927)
    #send_init_pose(simulator_process)
    #~ #generation d'un message de commande de deplacement
    #~ # "MSG_MAIN_GOTO 0:'ROTATION'/1:'DEPLACEMENT'/2:'DEPLACEMENT_LIGNE_DROITE' 0:'MARCHE_AVANT'/1:'MARCHE_ARRIERE'"
    #deplacement = commandMsg("MSG_MAIN_GOTO 1 1")   # 'DEPLACEMENT' en 'MARCHE_AVANT'
    #~ deplacement.addPose(str(d_cfgTraj['Distance']) + " 0.0 0.0 0") # deplacement rectiligne sur la distance d_cfgTraj['Distance']
    #deplacement.addPose("0.3 1.0 0.0 1")
    #transmission de commandes de deplacement par l'entree standard
    #simulator_process.stdin.write(deplacement.cmdMsgGeneration())

    #~ #Test rotation
    #~ deplacement = commandMsg("MSG_MAIN_GOTO 0 1")
    #~ deplacement.addPose("0.31 2.823 -1.5808 0")

    #3.1415927
    #~ deplacement = commandMsg("MSG_MAIN_GOTO 2 1")   # 'DEPLACEMENT_LIGNE_DROITE' en 'MARCHE_AVANT'
    #~ deplacement.addPose("0.310000002384 0.589999973774 -1.57079637051 1.0")

#~ #******************* Test ***********************************************************
    #~ #INIT_POSE_ROBOT 0 0 1 0.310000002384 2.82299995422 1.57079637051 1.0
    #~ send_init_pose(simulator_process, x=0.310000002384, y=2.82299995422, angle=1.57079637051)
    #~
    #~ #MSG_MAIN_GOTO 2 1 1 0.310000002384 2.41000008583 1.57079637051 1.0
    #~ deplacement = commandMsg("MSG_MAIN_GOTO 2 -1")
    #~ deplacement.addPose("0.310000002384 2.41000008583 1.57079637051 1")
    #~ #transmission de commandes de deplacement par l'entree standard
    #~ simulator_process.stdin.write(deplacement.cmdMsgGeneration())
    #~
    #~ #MSG_MAIN_GOTO 0 1 1 0.31468001008 2.40945506096 -3.14159274101 1.0
    #~ deplacement = commandMsg("MSG_MAIN_GOTO 0 1")
    #~ deplacement.addPose("0.31468001008 2.40945506096 -3.14159274101 1")
    #~ #transmission de commandes de deplacement par l'entree standard
    #~ simulator_process.stdin.write(deplacement.cmdMsgGeneration())
    #~
    #~ #MSG_MAIN_GOTO 2 1 1 1.03999996185 2.41000008583 3.06248354912 1.0
    #~ deplacement = commandMsg("MSG_MAIN_GOTO 2 -1")
    #~ deplacement.addPose("1.03999996185 2.41000008583 3.06248354912 1")
    #~ #transmission de commandes de deplacement par l'entree standard
    #~ simulator_process.stdin.write(deplacement.cmdMsgGeneration())

#~ #******************* Test ***********************************************************
    #~ #INIT_POSE_ROBOT 0 0 1 0.5 1.10000002384 0.259999990463 1.0
    #~ send_init_pose(simulator_process, x=0.5, y=1.10000002384, angle=0.259999990463)
    #~
    #~ #MSG_MAIN_GOTO 1 1 1 0.310000002384 2.29999995232 1.57079637051 1.0
    #~ deplacement = commandMsg("MSG_MAIN_GOTO 1 1")
    #~ deplacement.addPose("0.310000002384 2.29999995232 1.57079637051 1")
    #~ #transmission de commandes de deplacement par l'entree standard
    #~ simulator_process.stdin.write(deplacement.cmdMsgGeneration())
    #~
    #~ #MSG_MAIN_GOTO 0 1 1 0.307770013809 2.29856991768 1.57079637051 1.0
    #~ deplacement = commandMsg("MSG_MAIN_GOTO 0 1")
    #~ deplacement.addPose("0.307770013809 2.29856991768 1.57079637051 1")
    #~ #transmission de commandes de deplacement par l'entree standard
    #~ simulator_process.stdin.write(deplacement.cmdMsgGeneration())

#~ #******************* Test ***********************************************************
    #~ #INIT_POSE_ROBOT 0 0 1 1.60000002384 1.10000002384 0.259999990463 1.0
    #~ send_init_pose(simulator_process, x=1.60000002384, y=1.10000002384, angle=0.259999990463)
    #~
    #~ #MSG_MAIN_GOTO 1 1 1 1.5 0.699999988079 1.57079637051 1.0
    #~ deplacement = commandMsg("MSG_MAIN_GOTO 1 1")
    #~ deplacement.addPose("1.5 0.699999988079 1.57079637051 1")
    #~ #transmission de commandes de deplacement par l'entree standard
    #~ simulator_process.stdin.write(deplacement.cmdMsgGeneration())

    #~ #INIT_POSE_ROBOT 0 0 1 1.60000002384 1.10000002384 0.259999990463 1.0
    send_init_pose(simulator_process, x=0.31, y=0.177, angle=-1.57079637051)
    #~
    #~ #MSG_MAIN_GOTO 1 1 1 1.5 0.699999988079 1.57079637051 1.0
    deplacement = commandMsg("MSG_MAIN_GOTO 1 -1")
    deplacement.addPose("0.35 0.21 0 0")
    deplacement.addPose("0.37 0.65 0 0")
    deplacement.addPose("0.53 0.81 0 0")
    deplacement.addPose("1.29 0.83 0 0")
    #~ #transmission de commandes de deplacement par l'entree standard
    simulator_process.stdin.write(deplacement.cmdMsgGeneration())

    #transmission de la commande de d'arret du simulateur
    (stdoutdata, stderrdata) = simulator_process.communicate("QUIT\n")

    print("Simulation terminee.")
    #traitement des donnees de stdout
    lines = lineForm(stdoutdata)
    d_traj = stdoutParser(lines)

    return d_traj

def testPI(d_cfgTraj) :

    #lancement du simulateur de deplacement
    print("Lancement du simulateur")
    simulator_process = subprocess.Popen('simulator_trajAsser.exe', shell=True, stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.PIPE)

    # envoie de la configuration du simulateur
    send_config_simulator(simulator_process, d_cfgTraj)
    print("Config envoyee")

    #generation d'un message d'execution du test PI
    cmdTestPI = commandMsg("MSG_TEST_PI")
    #transmission de commandes de deplacement par l'entree standard
    simulator_process.stdin.write(cmdTestPI.cmdMsgGeneration())
    print("Test PI lance")

    #transmission de la commande d'arret du simulateur
    (stdoutdata, stderrdata) = simulator_process.communicate("QUIT\n")
    print("Simulation terminee.")

    #traitement des donnees de stdout
    lines = lineForm(stdoutdata)
    d_traj = stdoutParser(lines)

    return d_traj

def PI2011_coutOptimParam(paramPI, d_cfgTraj_local) :
    d_cfgTraj_local['Kp'] = paramPI[0]
    d_cfgTraj_local['Ki'] = paramPI[1]
    d_traj = testPI(d_cfgTraj_local)

    sommeErreurVitesseCarre = 0.0
    tensionMax = 0.0
    # cout de la tension maximum
    for tension in d_traj["tensionPWM_MoteurGauche"]:
        if (tension > tensionMax) :
            tensionMax = tension
    coutTensionMax = ((1023-20) - tensionMax) / (1023-20)
    if (coutTensionMax < 0.0) :
        coutTensionMax = -10.0*coutTensionMax

    # cout de l'erreur de vitesse en regime permanent
    indexMesureErrVitesse = len(d_traj["erreurVitesseMoteurGauche"]) - 1
    vitcons_p = d_traj["vitconsPI"][0]
    for index_vitcons in range(len(d_traj["vitconsPI"][1:])) :
        if ((d_traj["vitconsPI"][1 + index_vitcons] - vitcons_p) < 0.001) :
            if (d_traj["vitconsPI"][1 + index_vitcons] > 0.05) :
                indexMesureErrVitesse = int((1 + index_vitcons)*1.0) + 25
                break
        vitcons_p = d_traj["vitconsPI"][1 + index_vitcons]

    if (indexMesureErrVitesse > (len(d_traj["erreurVitesseMoteurGauche"]) - 1) ) :
        indexMesureErrVitesse = len(d_traj["erreurVitesseMoteurGauche"]) - 1

    coutErreurVitesse = math.fabs(d_traj["erreurVitesseMoteurGauche"][indexMesureErrVitesse] / d_traj["vitesseMoteurGauche"][indexMesureErrVitesse])

    coutTotal = coutTensionMax + coutErreurVitesse

    print([tensionMax, coutErreurVitesse, indexMesureErrVitesse, d_cfgTraj_local['Kp'], d_cfgTraj_local['Ki']])
    return coutTotal

def coutOptim_TempsAcc(param, d_cfgTraj_local) :
    d_cfgTraj_local['TempsAcc'] = param[0]
    #~ gainPI = PI2011_optimParam(d_cfgTraj_local)
    d_cfgTraj_local['Kp'] = param[1]
    d_cfgTraj_local['Ki'] = param[2]
    d_traj = testPI(d_cfgTraj_local)

    sommeErreurVitesseCarre = 0.0
    tensionMax = 0.0
    # cout de la tension maximum
    for tension in d_traj["tensionPWM_MoteurGauche"]:
        if (tension > tensionMax) :
            tensionMax = tension
    coutTensionMax = ((1023-20) - tensionMax) / (1023-20)
    if (coutTensionMax < 0.0) :
        coutTensionMax = -10.0*coutTensionMax

    # cout de l'erreur de vitesse en regime permanent
    indexMesureErrVitesse = len(d_traj["erreurVitesseMoteurGauche"]) - 1
    vitcons_p = d_traj["vitconsPI"][0]
    for index_vitcons in range(len(d_traj["vitconsPI"][1:])) :
        if ((d_traj["vitconsPI"][1 + index_vitcons] - vitcons_p) < 0.001) :
            if (d_traj["vitconsPI"][1 + index_vitcons] > 0.05) :
                indexMesureErrVitesse = int((1 + index_vitcons)*1.0) + 25
                break
        vitcons_p = d_traj["vitconsPI"][1 + index_vitcons]

    if (indexMesureErrVitesse > (len(d_traj["erreurVitesseMoteurGauche"]) - 1) ) :
        indexMesureErrVitesse = len(d_traj["erreurVitesseMoteurGauche"]) - 1

    #~ coutErreurVitesse = 0.0
    #~ for index_errVit in range(len(d_traj["erreurVitesseMoteurGauche"][indexMesureErrVitesse:])) :
         #~ errRel = math.fabs(d_traj["erreurVitesseMoteurGauche"][indexMesureErrVitesse+index_errVit] / d_traj["vitesseMoteurGauche"][indexMesureErrVitesse+index_errVit])
         #~ if (errRel < 0.0) :
             #~ errRel = -1.0*errRel
         #~ coutErreurVitesse = coutErreurVitesse + errRel
    #~ coutErreurVitesse = coutErreurVitesse / len(d_traj["erreurVitesseMoteurGauche"][indexMesureErrVitesse:])

    coutErreurVitesse = math.fabs(d_traj["erreurVitesseMoteurGauche"][indexMesureErrVitesse] / d_traj["vitesseMoteurGauche"][indexMesureErrVitesse])


    # cout de l'erreur de vitesse en regime permanent
    indexParcours = len(d_traj["distPI"]) - 1

    for index_distPI in range(len(d_traj["distPI"])) :
        if (d_traj["distPI"][index_distPI] > 1.0) :
            indexParcours = index_distPI
            break
    tempsParcours = (indexParcours*d_traj["periode"][0])

    coutTotal = coutTensionMax*2.0 + coutErreurVitesse + tempsParcours* 6.0

    print([tensionMax, coutErreurVitesse, indexMesureErrVitesse*0.02, indexParcours, d_cfgTraj_local['TempsAcc'], d_cfgTraj_local['Kp'], d_cfgTraj_local['Ki']])

    return coutTotal

def affichageTraj2011(d_traj):
    matplotlib.rcParams.update({'font.size': 10})
    
    print("duree: " + str(sum(d_traj["time"])) + "s")
    periode = d_traj['periode'][0]

    #nombre de variable loguer
    print("Nombre de variable loguer: " + str(len(d_traj.keys())))
    #~ print("CPTvitesseAnglePtArret: " + str(d_traj["CPTvitesseAnglePtArret"]))
    #~ print("vitesseAnglePtArret: " + str(d_traj["vitesseAnglePtArret"]))
    if (d_traj.keys().count('finAsser') == 1):
        print("Test fin asser OK")
        print(d_traj['finAsser'])
    else : print("Test fin asser negatif")

    print("len_xRoueDroite: " + str(len(d_traj["xRoueDroite"])))
    print(len(d_traj["xRoueGauche"]))

    figure(1)
    plot(d_traj["xRoueGauche"], d_traj["yRoueGauche"], label="Rgauche")
    hold(True)
    plot(d_traj["xRoueDroite"], d_traj["yRoueDroite"], label="Rdroite")
    plot(d_traj["xPoseReferenceRobot"], d_traj["yPoseReferenceRobot"], '--r', label="TrajRref")
    xCentre = [(d_traj["xRoueGauche"][index] + d_traj["xRoueDroite"][index])/2.0 for index in range(len(d_traj["xRoueDroite"]))]
    yCentre = [(d_traj["yRoueGauche"][index] + d_traj["yRoueDroite"][index])/2.0 for index in range(len(d_traj["yRoueDroite"]))]
    plot(xCentre, yCentre, '-m', label="Rcentre")
    print("errDistFinale : " + str(math.sqrt(math.pow(0.6 - xCentre[-1], 2.0) + math.pow(0.4 - yCentre[-1], 2.0))) + ", " + str(0.6 - xCentre[-1]))
    posIndex = 70
    print("angle_final: " + str(d_traj["angle"][-1]))

    #~ plot((d_traj["xRoueGauche"][posIndex]+d_traj["xRoueDroite"][posIndex])/2.0, (d_traj["yRoueGauche"][posIndex]+d_traj["yRoueDroite"][posIndex])/2.0, 'ro')
    grid(True)
    xlabel("x (m)")
    ylabel("y (m)")
    legend(loc='lower right')
    title("trajectoire")

    ### Affichage des donnees de vitesses ###
    temps = [index*periode for index in range(len(d_traj["vitesseMoteurGauche"]))]
    print("len temps: " + str(len(temps)))
    print("len VitesseProfil: " + str(len(d_traj["VitesseProfil"])))
    figure(2)
    #setp( ax1.get_xticklabels(), fontsize=6)

    ax_v = subplot(211)
    ylabel("vitesse (m/s)")
    plot(temps, d_traj["vitesseMoteurGauche"], '-', label='mesure gauche')
    # hold(True)
    plot(temps, d_traj["vitesseMoteurDroit"], label='mesure droit')
    plot(temps, d_traj["VitesseProfil"], label='Vitesse Profil')
    plot(temps, d_traj["ConsigneMoteurDroit_MS"], label='consigne droit')
    plot(temps, d_traj["VgASR"], label='V g ASR')
    plot(temps, d_traj["Phase"], label='Phase')
    plot(temps, d_traj["fFin"], label='flag Fin')
    plot(temps, d_traj["vitLongitudinale"], '--', label='consigne vit long')
    #~ plot(temps, d_traj["vitLongMvt"], '-', label='consigne vit long Mvt')
    
    #~ plot(temps, d_traj["vitLongitudinaleEffective"], '--', label='consigne vit long effec')
    print(str(len(d_traj["VpiD"])) + " " + str(len(d_traj["VitesseProfil"]))) 
    #~ plot(temps, d_traj["VposG"], label='VposG')
    #~ plot(temps, d_traj["VposD"], label='VposD')
    # plot(temps, d_traj["VpiG"], label='VpiG')
    # plot(temps, d_traj["VpiD"], label='VpiD')
    
    # plot([index*periode for index in range(len(d_traj["ConsigneMoteurDroit"]))], [(cons-1024.0)*0.000985 for cons in d_traj["ConsigneMoteurDroit"]], label='consigne droit')
    # plot([index*periode for index in range(len(d_traj["vitesseProfilConsigne"]))], d_traj["vitesseProfilConsigne"], 'o', label='vitesse profil')
    # plot([index*periode for index in range(len(d_traj["vitLongitudinale"]))], d_traj["vitLongitudinale"], label='consigne long calculee')
    # plot([index*periode for index in range(len(d_traj["parametrePositionSegmentTrajectoire"]))], d_traj["parametrePositionSegmentTrajectoire"], label='paramTraj')
    # plot([index*periode for index in range(len(d_traj["xCCourant"]))], d_traj["xCCourant"], 'o', label='xCCourant')
    # plot([index*periode for index in range(len(d_traj["xPoseReferenceRobot"]))], d_traj["xPoseReferenceRobot"], 'o', label='xPoseReferenceRobot')
    # plot([index*periode for index in range(len(d_traj["distNormaliseeRestante"]))], d_traj["distNormaliseeRestante"], 'o', label='distNormaliseeRestante')
    # plot([index*periode for index in range(len(d_traj["distance_restante"]))], d_traj["distance_restante"], 'o', label='distance_restante')
    # plot([index*periode for index in range(len(d_traj["etat"]))], d_traj["etat"], label='etat')
    # plot([index*periode for index in range(len(d_traj["limit"]))], d_traj["limit"], 'om', label='limit')
    # plot([index*periode for index in range(len(d_traj["distance_decceleration"]))], d_traj["distance_decceleration"], '-', label='distDecc')

    # plot([index*periode for index in range(len(d_traj["ASSER_Running"]))], d_traj["ASSER_Running"], 'o', label='ASSER_Running')

    #plot([index*periode for index in range(len(d_traj["val_tab_vit"]))], d_traj["val_tab_vit"], '-', label='vitesse profil')

    if "periodeNewSeg" in traj.keys() :
        for tpsNewSeg in traj["periodeNewSeg"] :
            plot([tpsNewSeg*periode, tpsNewSeg*periode], [-0.6, 0.6], '-k')

    # print("decc_tempsAcc_float: " + str(d_traj["decc_tempsAcc_float"]))
    # print("decc_tempsAcc: " + str(d_traj["decc_tempsAcc"]))
    # print("decc_vmax: " + str(d_traj["decc_vmax"]))

    ylim(-1.2, 1.2)
    #~ legend(loc="lower center")
    grid(True)
    title("vitesses")
#~ 
    #figure(3)
    ax_t = subplot(212, sharex=ax_v)
    ylabel("tension (unite PWM)")
    plot(temps, d_traj["tensionPWM_MoteurGauche"], label='moteur gauche')
    # hold(True)
    plot(temps, d_traj["tensionPWM_MoteurDroit"], label='moteur droit')
    
    if "periodeNewSeg" in traj.keys() :
        for tpsNewSeg in traj["periodeNewSeg"] :
            plot([tpsNewSeg*periode, tpsNewSeg*periode], [-1000.0, 1000.0], '-k')
    
    legend(loc="lower center")
    grid(True)
    title("tensions moteurs")


    #~ figure(5)
    #~ plot(traj["dist_parcourue"], d_traj["vitLongitudinaleEffective"], 'o', label="consigne vit longitudinale effective")
    #~ plot(traj["dist_parcourue"][:-1], d_traj["vitLongitudinaleTest"], '-o', label="consigne vit longitudinale test")
    
    #~ plot(d_traj["dist_parcourue"], label="dist_p")
    #~ plot(d_traj["paramPoseSubSegCourant"], label="param")
    
    #plot(d_traj["index_get_vitesseGabarit"])
    #~ plot(d_traj["index_tab_vit"])
    #~ plot(d_traj["dist_parcourue"], d_traj["val_tab_vit"])
    #~ plot(d_traj["val_tab_vit"])
#    plot([index*periode for index in range(len(d_traj["dist_parcourue"]))], d_traj["dist_parcourue"])
    #~ plot(d_traj["angleRobot"], '-o')
    #~ plot(d_traj["angleRef"], '-o')
    #~ plot(d_traj["nbdepas"])
    #~ plot(d_traj["poseRefX"], 'o', label="poseRefX")
    #~ plot(d_traj["poseRobotX"], 'x', label="poseRobotX")
    #~ plot(d_traj["poseRefY"], 'o', label="poseRefY")
    #~ plot(d_traj["poseRobotY"], label="poseRobotY")
    #~ print("poseRobotInitX: " + str(d_traj["poseRobotInitX"]))
    #~ print("poseRobotInitY: " + str(d_traj["poseRobotInitY"]))
    #~ plot(d_traj["thetaPoseReference"], label="thetaPoseReference")
    
    legend()
    grid(True)
    #~ title("Distance parcourue")

    figure(3)
    ax_err_x = subplot(311)
    ylabel("erreur x (mm)")
    plot(temps, [x*1e3 for x in d_traj["erreurPose_x"]])
    grid(True)
    
    ax_err_y = subplot(312)
    ylabel("erreur y (mm)")
    plot(temps, [y*1e3 for y in d_traj["erreurPose_y"]])
    grid(True)
    
    ax_err_x = subplot(313)
    ylabel("erreur angle (degre)")
    plot(temps, [angle_rad*180.0/math.pi for angle_rad in d_traj["erreurPose_angle"]])
    grid(True)

    if "dist_parcourue_rot" in d_traj.keys() :
        figure()
        plot([d_traj["distance"] for d in d_traj["dist_parcourue_rot"]], label="distTotRot")
        plot(d_traj["dist_parcourue_rot"], label="distParcRot")
        grid(True)
        legend(loc="center right")

    #~ affichageGabaritVitesse(d_traj)

    show()

def affichageTestAccDcecc(d_traj):
    print("duree: " + str(d_traj["time"][0]) + "s")
    periode = d_traj['periode'][0]

    #nombre de variable loguer
    print("Nombre de variable loguer: " + str(len(d_traj.keys())))
    #~ print("CPTvitesseAnglePtArret: " + str(d_traj["CPTvitesseAnglePtArret"]))
    #~ print("vitesseAnglePtArret: " + str(d_traj["vitesseAnglePtArret"]))
    if (d_traj.keys().count('finAsser') == 1):
        print("Test fin asser OK")
        print(d_traj['finAsser'])
    else : print("Test fin asser negatif")

    print(len(d_traj["xRoueDroite"]))

    temps = [index*periode for index in range(len(d_traj["vitesseMoteurGauche"]))]
    figure(2)
    subplot(2,1,1)
    plot(temps, d_traj["vitesseMoteurGauche"], '-', label='mesure gauche')
    hold(True)
    plot(temps, d_traj["vitesseMoteurDroit"], label='mesure droit')


    plot([index*periode for index in range(len(d_traj["val_tab_vit"]))], d_traj["val_tab_vit"], '-', label='vitesse profil')

    #~ print("decc_tempsAcc_float: " + str(d_traj["decc_tempsAcc_float"]))
    #~ print("decc_tempsAcc: " + str(d_traj["decc_tempsAcc"]))
    #~ print("decc_vmax: " + str(d_traj["decc_vmax"]))

    v = axis()
    limits = []
    for val in v:
        limits.append(val)
    limits[2] = -1.2
    limits[3] = 1.2
    axis(limits)
    legend(loc="lower center")
    grid(True)
    title("vitesses")

    #~ figure(3)
    subplot(2,1,2)
    plot(temps, d_traj["tensionPWM_MoteurGauche"], label='moteur gauche')
    hold(True)
    plot(temps, d_traj["tensionPWM_MoteurDroit"], label='moteur droit')
    legend(loc='lower center')
    grid(True)
    title("tensions moteurs")

    figure(5)
    plot(d_traj["index_get_vitesseGabarit"])

    legend()

    #~ affichageGabaritVitesse(d_traj)

    show()

def affichageTestPI2012(d_traj):

    periode=d_traj["periode"][0]
    temps = [ periode * x for x in range(len(d_traj["vitesseMoteurGauche"]))]
    print("len: " + str(len(d_traj["vitesseMoteurGauche"])) )

    figure(1)
    plot(temps, d_traj["tensionPWM_MoteurGauche"], label="tension moteur gauche")
    hold(True)
    #~ plot(temps, d_traj["tensionPWM_MoteurDroit"], label="tension moteur droit")
    grid(True)
    legend(loc="lower center")
    title("tensions moteurs")

    figure(2)
    plot(temps, d_traj["vitconsPI"], label="vitconsPI")
    plot(temps, d_traj["vitesseMoteurGauche"], label="vitesseG")
    grid(True)
    legend(loc="lower center")

    figure(3)
    plot(temps, d_traj["distPI"], label="distance parcourue")
    grid(True)
    legend(loc="upper left")
    title("Distance parcourue")

    print("dist finale: " + str(d_traj["distPI"][-1]))


    v = axis()
    limits = []
    for val in v:
        limits.append(val)
    #~ limits[2] = -1.5
    #~ limits[3] = 1.5
    axis(limits)

    show()

def affichageGabaritVitesse_2012(d_traj):

    figure(4)
    # affichage avec un pas de 2cm
    pas = d_traj["pas_ech"][0]
    print("pas ech: " + str(pas))
    plot([index * pas for index in range(len(d_traj["gabarit_vitesse"]))], d_traj["gabarit_vitesse"], '-o', label='vit')
    plot([index * pas for index in range(len(d_traj["gabarit_acceleration"]))], d_traj["gabarit_acceleration"], '-o', label='acc')
    if (d_traj.keys().count("gabarit_acceleration_new")) :
        plot([index * pas for index in range(len(d_traj["gabarit_acceleration_new"]))], d_traj["gabarit_acceleration_new"], '-o', label='acc2')
    if (d_traj.keys().count("gabarit_vitesse_new")) :
        plot([index * pas for index in range(len(d_traj["gabarit_vitesse_new"]))], d_traj["gabarit_vitesse_new"], '-o', label='vit2')
    
    #~ plot([index * pas for index in range(len(d_traj["gabarit_delta_acceleration"]))], d_traj["gabarit_delta_acceleration"], '-om', label='delta')
    #~ plot(d_traj["gabarit_acceleration_max"], [0 for index in range(len(d_traj["gabarit_acceleration_max"]))], '-or')
    print("taille tab gabarit vitesse : " + str(len(d_traj["gabarit_vitesse"])))
    #~ print("nb acc max : " + str(len(d_traj["gabarit_acceleration_max"])))
    #~ print("init_gabarit:")
    #~ for val in d_traj["init_gabarit"] :
        #~ print(val)
    grid()
    title("Gabarit vitesse")
    legend()

    #~ print("test sum_acc:")
    #~ for sum_acc in d_traj["gabarit_delta_acceleration"] :
        #~ print(sum_acc)

    #~ figure(6)
    #~ plot([index * pas for index in range(len(d_traj["gabarit_delta_acceleration"]))], d_traj["gabarit_delta_acceleration"], 'o')
    #~ grid()
    
def affichageGabaritVitesse_2013(d_traj):

    figure(4)
    
    pas = d_traj["pas_ech"][0]
    print("pas ech: " + str(pas))
    
    plot([index * pas for index in range(len(d_traj["gabarit_vitesse"]))], d_traj["gabarit_vitesse"], '-o', label='vitesse limite')
    #~ plot([index * pas for index in range(len(d_traj["gabarit_acceleration"]))], d_traj["gabarit_acceleration"], '-o', label='acc')
    
    #~ if (d_traj.keys().count("gabarit_acceleration_new")) :
        #~ plot([index * pas for index in range(len(d_traj["gabarit_acceleration_new"]))], d_traj["gabarit_acceleration_new"], '-o', label='acc2')
    #~ if (d_traj.keys().count("gabarit_vitesse_new")) :
        #~ plot([index * pas for index in range(len(d_traj["gabarit_vitesse_new"]))], d_traj["gabarit_vitesse_new"], '-o', label='vit2')
        
    print(len(traj["dist_parcourue"]))
    print(len(d_traj["vitLongitudinaleEffective"]))
    print(len(d_traj["VitesseReelleMesure"]))
    print(len(traj["VitesseProfil"]))
    print(len(traj["distanceParcourue_Profil"]))
    plot(traj["dist_parcourue"], d_traj["vitLongitudinaleEffective"], '-o', label="consigne vit longitudinale effective")
    plot(traj["dist_parcourue"], d_traj["VitesseReelleMesure"], '-o', label="mesure vit longitudinale")
    plot(traj["dist_parcourue"], d_traj["VitesseProfil"], '-o', label='Vitesse Profil')
    #~ plot(traj["dist_parcourue"], d_traj["distanceParcourue_Profil"], '-o', label='"distanceParcourue_Profil"')
    
    print("taille tab gabarit vitesse : " + str(len(d_traj["gabarit_vitesse"])))

    grid()
    title("Plafond de vitesse sur toute la trajectoire")
    legend(loc="lower center")
    xlabel("distance")
    ylabel("vitesse max en multipoint")
    
    v = axis()
    limits = []
    for val in v:
        limits.append(val)
    limits[2] = 0.0
    #~ limits[3] = 1.2
    axis(limits)
    
    #~ figure(5)
    #~ plot(traj["f_T"], 'o')
    #~ lx = range(15)
    #~ a = (0.03 - 0.1176) / 225
    #~ b = -30*a
    #~ c = 0.03
    #~ plot(lx, [a*math.pow(x,2.0) + b*x +c for x in lx], '-')


def optimRayonRoue(d_cfgTraj_local) :
    l_simus = []
    l_Distance = [1.0] #numpy.arange(0.4, 0.5, 0.2)
    for Distance in l_Distance :
        d_cfgTraj_local['Distance'] = Distance
        l_R = numpy.arange(0.04, 0.12, 0.02)
        l_duree = []
        for R in l_R :
            d_cfgTraj_local['Rayon_roue'] = R
            #optimisation des gains PI
            optimParam_TempsAcc(d_cfgTraj_local)
            #simulation
            traj = trajFunction(d_cfgTraj_local)

            l_duree.append(traj["time"][0])
        l_simus.append(l_duree)

    print("fin simus")

    figure(1)
    for simu in l_simus :
        plot(l_R, simu, '-o')
    grid()
    show()

def PI2011_optimParam(d_cfgTraj_local) :
    #optimisation des gains PI
    p0 = [2.0, 2.0]
    solPI = fmin(PI2011_coutOptimParam, p0, args=[d_cfgTraj_local], xtol=0.01, ftol=0.001, maxiter=45, disp = True, retall = True)
    return(solPI[0])

def optimParam_TempsAcc(d_cfgTraj_local) :
    #optimisation du temps TempsAcc et des gains PI
    p0 = [0.5, 1.0, 2.0] # [TempsAcc, Kp, Ki]
    sol = fmin(coutOptim_TempsAcc, p0, args=[d_cfgTraj_local], xtol=0.01, ftol=0.001, maxiter=45, disp = True, retall = True)

    d_cfgTraj_local["TempsAcc"] = sol[0][0]
    d_cfgTraj_local["Kp"] = sol[0][1]
    d_cfgTraj_local["Ki"] = sol[0][2]
    #~ affichageTestPI2012(testPI(d_cfgTraj_local))

    return(d_cfgTraj_local)

def printLog(traj, logName) :
    if logName in traj.keys() :
        print(logName + " : " + str(traj[logName]))
        

#########################################################################
#~ traj = testPI(d_cfgTraj)
#~ affichageGabaritVitesse_2012(traj)
#~ affichageTestPI2012(traj)
#~ sys.exit(2)

#~ optimRayonRoue(d_cfgTraj)
#~ sys.exit(2)

#~ coutTA = coutOptim_TempsAcc(0.3, d_cfgTraj)
#~ print(coutTA)
#~ sys.exit(2)

#~ optimParam_TempsAcc(d_cfgTraj)
#~ sys.exit(2)

#~ gainPI = PI2011_optimParam(d_cfgTraj)
#~ print("Gains PI: " + str(gainPI))
#~ sys.exit(2)

#~ d_cfgTraj = optimParam_TempsAcc(d_cfgTraj)
traj = trajFunction(d_cfgTraj)
print("Nb var log : " + str(len(traj.keys())))

#~ traj = trajTest(d_cfgTraj)

#~ affichageGabaritVitesse_2013(traj)
#~ show()
#~ sys.exit(2)
#~ *********************************************************************

#~ print(len(traj["def_xTraj"]))

#~ print([traj["def_xTraj"][0], traj["def_yTraj"][0]])
#~ print([traj["def_xTraj"][1], traj["def_yTraj"][1]])
#~ print([traj["def_xTraj"][2], traj["def_yTraj"][2]])
#~ print([traj["def_xTraj"][-1], traj["def_yTraj"][-1]])


#~ print("pti:")
#~ print(traj["pti"])
#~ print("disti")
#~ print(traj["disti"])
#~ print("distance des segments :") ######################
#~ print(traj["distance_seg"])
#~ print("distance: ") ####################
#~ print(traj["distance"])
#~ print("intervalle: ")
#~ print(traj["intervalle"])
#~ print("theta1: " + str(traj["theta1"]))
#~ print("angle_rad: " + str(traj["angle_rad"]))

#~ 
    
    #~ 
    #~ figure()
    #~ plot(traj["def_angleTraj"], '-o', label="thetaTraj")
    #~ legend()
    #~ 
    #~ figure()
    #~ plot(traj["def_diff_xTrASSER_TRAJ_LogAsserValPC("distance_seg", chemin.segmentTrajBS[iSegment].distance);aj"], '-o', label="diff_x")
    #~ plot(traj["def_diff_yTraj"], '-o', label="diff_y")
    #~ legend()
    #~ grid()
    #~ 
    #~ figure()
    #~ plot(traj["def_diff2_xTraj"], '-o', label="diff2_x")
    #~ plot(traj["def_diff2_yTraj"], '-o', label="diff2_y")
    #~ legend()
    #~ grid()
    #~ 
    #~ figure()
    #~ plot(traj["def_diff_ThetaTraj"], '-o', label="diff_Theta")
    #~ legend()
    #~ grid()


#~ print("angle_init: " +str(traj["angleFinRotation"][0]*180.0/math.pi))
#~ print("angleFinRotation: " +str(traj["angleFinRotation"][0]*180.0/math.pi))
#~ print("test_modulo_angle: " +str(traj["angleFinRotation"][2]))
#~ print("plageAngleRotation: " +str(traj["plageAngleRotation"][0]*180.0/math.pi))

#~ print("distance_seg: " + str(traj["distance_seg"][0]))

#~ figure()
#~ plot([theta*180.0/math.pi for theta in traj["angle"]])
#~ plot(traj["consigneRotation"], label="consRot")

#~ figure()
#~ plot([theta*180.0/math.pi for theta in traj["angle"]])
#~ plot(traj["diffAngle"], label="A")
#~ plot(traj["diffX"], label='X')
#~ plot(traj["diffY"], label='Y')
#~ legend()

#~ figure()
#~ plot(traj["consRotation"], '-o', label="consRot")
#~ legend()
#~ grid()

#~ print("debug_smooth:")
#~ print(traj["debug_smooth"])

if "vitesse_fin_profil" in traj.keys() :
    print("vitesse debut et fin profil (m/s):")
    print(traj["vitesse_debut_profil"])
    print(traj["vitesse_fin_profil"])

if "dist_parcourue" in traj.keys() :
    print("dist_parcourue: " + str(traj["dist_parcourue"][-1]))
    
if "Motion" in traj.keys() :
    print("etat asser: " + str(traj["Motion"][0]))

print("Nb msg err : " + str(len(traj["message"])))
for msg in traj['message'] :
    print(msg)
  
if "periodeNewSeg" in traj.keys() :  
    print("periodeNewSeg : " + str(traj["periodeNewSeg"]))
    
### Debug new traj ###
if "use_angle" in traj.keys() :  
    print("use_angle : " + str(traj["use_angle"]))
    
if "NbSegCurve" in traj.keys() :  
    print("NbSegCurve : " + str(traj["NbSegCurve"]))
    
if "flag_subSeg_used" in traj.keys() :
    print("flag_subSeg_used : " + str(traj["flag_subSeg_used"]))
    
if "subSegClass" in traj.keys() :
    print("subSegClass : " + str(traj["subSegClass"]))

if "segConfigured" in traj.keys() :
    print("segConfigured : " + str(traj["segConfigured"]))

#~ printLog(traj, "data_sds_ab")
printLog(traj, "angle_rad")

#~ printLog(traj, "data_sds_ab_base")

#~ NbV = 12
#~ N = int(len(traj["data_sds_ab_base"]) / (NbV*1.0))
#~ for n in range(N) :
    #~ print("data_sds_ab_base : \n ax1: {0} \n bx1: {1} \n ay1: {2} \n by1: {3} \n qx1: {4} \n qy1: {5} \n ax2: {6} \n bx2: {7} \n ay2: {8} \n by2: {9} \n qx2: {10} \n qy2: {11}".format(traj["data_sds_ab_base"][n*NbV], traj["data_sds_ab_base"][n*NbV + 1], traj["data_sds_ab_base"][n*NbV + 2], traj["data_sds_ab_base"][n*NbV + 3], traj["data_sds_ab_base"][n*NbV + 4], traj["data_sds_ab_base"][n*NbV + 5], traj["data_sds_ab_base"][n*NbV + 6], traj["data_sds_ab_base"][n*NbV + 7], traj["data_sds_ab_base"][n*NbV + 8], traj["data_sds_ab_base"][n*NbV + 9], traj["data_sds_ab_base"][n*NbV + 10], traj["data_sds_ab_base"][n*NbV + 11]))


#~ NbV = 4
#~ N = int(len(traj["data_sds_ab"]) / (NbV*1.0))
#~ for n in range(N) :
    #~ print("data_sds_ab : \n qx1: {0} \n qy1: {1} \n qx2: {2} \n qy2: {3}".format(traj["data_sds_ab"][n*NbV], traj["data_sds_ab"][n*NbV + 1], traj["data_sds_ab"][n*NbV + 2], traj["data_sds_ab"][n*NbV + 3]))
    #~ 
#~ NbV = 3
#~ N = int(len(traj["data_Rotation_config_arc1"]) / (NbV*1.0))
#~ for n in range(N) :
    #~ print("data_Rotation_config_arc1 : \n angle: {0} \n qx1: {1} \n qy1: {2}".format(traj["data_Rotation_config_arc1"][n*NbV], traj["data_Rotation_config_arc1"][n*NbV + 1], traj["data_Rotation_config_arc1"][n*NbV + 2]))

#~ printLog(traj, "data_sds_ab_base_A")

#~ print(traj["test_x"])
printLog(traj, "test_x_init")
#~ print("cpt xtraj : {0}".format(len(traj["def_xTraj"])))
#~ print("cpt test x : {0}".format(len(traj["test_x"])))
printLog(traj, "angle_r1")
printLog(traj, "angle_r2")
printLog(traj, "curvature_forced_2_prec")
printLog(traj, "inflexion_point")
printLog(traj, "inflexion_point_theta_seg")
printLog(traj, "distance_seg")
#~ printLog(traj, "poseReferenceRobot")

#~ printLog(traj, "xRoueDroite")
#~ printLog(traj, "yRoueDroite")
#~ printLog(traj, "vitesseMoteurDroit")
#~ printLog(traj, "paramPoseSubSegCourant")
#~ printLog(traj, "ASSER_Running")
#~ print("Nb Periode (len ASSER_Running) : " + str(len(traj["ASSER_Running"])))

#~ if "fin_MVT" in traj.keys() : 
    #~ print("fin_MVT : " + str(len(traj["fin_MVT"])))
#~ else :
    #~ print("fin_MVT : no log")
#~ printLog(traj, "fin_MVT")

#~ printLog(traj, "xPoseReferenceRobot")

#~ printLog(traj, "VitesseProfil")
#~ printLog(traj, "distanceTotale_Profil")

printLog(traj, "temps_init")
printLog(traj, "chemin_arc")
printLog(traj, "theta0_arc")
printLog(traj, "angle_arc")
#~ printLog(traj, "cfgAsser")
printLog(traj, "distSupp")
printLog(traj, "resetProfil")
printLog(traj, "ASSER_Running_TvF")
    
matplotlib.rcParams.update({'font.size': 16})

#~ if "def_xTraj" in traj.keys() :
    #~ figure(10)
    #~ N = len(traj["def_xTraj"])
    #~ plot(traj["def_xTraj"][:N], traj["def_yTraj"][:N], 'o')
    #~ grid()
    #~ 
    #~ figure(11)
    #~ plot(traj["def_Rinv"], '-o')
    #~ title('Rinv')
    #~ grid()
    #~ 
    #~ figure(12)
    #~ plot(traj["def_vLim"], '-o')
    #~ title('vitesse limite')
    #~ grid()
    #~ 
    #~ figure(13)
    #~ plot(traj["def_dl_dt"], '-o')
    #~ title('dl_dt')
    #~ grid()
#~ else :
    #~ print("No traj def plots")

# affichageGabaritVitesse_2013(traj)

affichageTraj2011(traj)


show()

#~ sys.exit(2)
