#!/usr/bin/python
# -*- coding: iso-8859-1 -*-


import subprocess
import sys
import time
from matplotlib.pyplot import *
import numpy
import math
import re
from scipy.optimize.optimize import fmin
from scipy.optimize import fmin_slsqp

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
    d_traj = {}
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
    poseInit = commandMsg("INIT_POSE_ROBOT 0 0")
    poseInit.addPose(str(x) + " " + str(y) + " " + str(angle) + " " + "0.0")
    return poseInit.cmdMsgGeneration()
    
def MSG_config_PI(Kp, Ki) :
    #generation d'un message de configuration des gains de l'asser de vitesse PI
    parametersPI = commandMsg("PARAMETERS_PI")
    parametersPI.addPose("KP" + " " + str(Kp))
    parametersPI.addPose("KI" + " " + str(Ki))
    return parametersPI.cmdMsgGeneration()
    
def MSG_config_AsserTrajectoire(K1, K2, K3) :
    #generation d'un message de configuration des gains de l'asser de déplacement longitudinal
    parametersK = commandMsg("PARAMETERS_GAIN_K")
    parametersK.addPose("GAIN_K1" + " " + str(K1))
    parametersK.addPose("GAIN_K2" + " " + str(K2))
    parametersK.addPose("GAIN_K3" + " " + str(K3))
    return parametersK.cmdMsgGeneration()
    
def MSG_config_AsserRotation(R1, R2) :
    #generation d'un message de configuration des gains de l'asser de rotation
    parametersR = commandMsg("PARAMETERS_GAIN_ROT")
    parametersR.addPose("GAIN_R1" + " " + str(R1))
    parametersR.addPose("GAIN_R2" + " " + str(R2))
    return parametersR.cmdMsgGeneration()
    
def MSG_config_profilVitesse(T_Acc, F_VA_Max, Umax) :
    #generation d'un message de configuration des parametres du profil de vitesse
    parametersT = commandMsg("PARAMETERS_TIME")
    parametersT.addPose("ACC" + " " + str(T_Acc))
    parametersT.addPose("VITANGMAX" + " " + str(F_VA_Max))
    parametersT.addPose("UMAX" + " " + str(Umax))
    return parametersT.cmdMsgGeneration()
    
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
    
def trajFunction(d_cfgTraj, Kp = 10.0, Ki = 5.0, K1 = 20.0, K2 = 75.0, K3 = 50.0, R1 = -10.2, R2 = -10.2, T1=2.0, T2=4.48, T3=900.0):
    #generation d'un message d'init de pose
    initPoseMessage = MSG_init_pose()
    
    #generation d'un message de configuration des gains de l'asser de vitesse PI
    msg_parametersPI = MSG_config_PI(d_cfgTraj['Kp'], d_cfgTraj['Ki'])
    
    #generation d'un message de configuration des gains de l'asser de déplacement longitudinal
    msg_parametersK = MSG_config_AsserTrajectoire(d_cfgTraj['K1'], d_cfgTraj['K2'], d_cfgTraj['K3'])
    
    #generation d'un message de configuration des gains de l'asser de rotation
    msg_parametersR = MSG_config_AsserRotation(d_cfgTraj['R1'], d_cfgTraj['R2'])
    
    #generation d'un message de configuration des parametres du profil de vitesse
    msg_parametersT = MSG_config_profilVitesse(d_cfgTraj['TempsAcc'], d_cfgTraj['Facteur_vitesse_angulaire'], d_cfgTraj['Umax'])
    
    #generation d'un message d'init des parametres du modele des moteurs
    msg_parametersMotor = MSG_config_modeleMoteurCC(d_cfgTraj['Masse'],
                                                    d_cfgTraj['Rayon_roue'],
                                                    d_cfgTraj['Frottement_fluide'],
                                                    d_cfgTraj['Force_resistante'],
                                                    d_cfgTraj['Resistance_induit'],
                                                    d_cfgTraj['Inductance_induit'],
                                                    d_cfgTraj['Constance_couple'],
                                                    d_cfgTraj['Constante_vitesse'],
                                                    d_cfgTraj['Rapport_reduction'])
    
    #generation d'un message de commande de deplacement
    # "MSG_MAIN_GOTO 0:'DEPLACEMENT'/1:'ROTATION' 0:'MARCHE_AVANT'/1:'MARCHE_ARRIERE'"
    dep = commandMsg("MSG_MAIN_GOTO 1 1")   # 'DEPLACEMENT' en 'MARCHE_AVANT'
    dep.addPose(str(d_cfgTraj['Distance']) + " 0.0 0.0 0") # deplacement rectiligne sur la distance d_cfgTraj['Distance']
    depMessage = dep.cmdMsgGeneration()
    
    #lancement du simulateur de deplacement
    print("Lancement du simulateur")
    simulator_process = subprocess.Popen('./simulator_trajAsser', shell=True, stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.PIPE)
    
    #configuration de l'asser de trajectoire
    simulator_process.stdin.write(msg_parametersPI)
    simulator_process.stdin.write(msg_parametersK)
    simulator_process.stdin.write(msg_parametersR)
    simulator_process.stdin.write(msg_parametersT)
    simulator_process.stdin.write(msg_parametersMotor)
    
    #transmission de l'init de pose par l'entree standard
    simulator_process.stdin.write(initPoseMessage)
    
    #transmission de commandes de deplacement par l'entree standard
    simulator_process.stdin.write(depMessage)
    
    #transmission de la commande de d'arret du simulateur
    (stdoutdata, stderrdata) = simulator_process.communicate("QUIT\n")
    
    print("Simulation terminee.")
    #traitement des donnees de stdout
    lines = lineForm(stdoutdata)
    d_traj = stdoutParser(lines)
    
    return d_traj

def testPI(d_cfgTraj):
    
    #generation d'un message de configuration des gains de l'asser de vitesse PI
    msg_parametersPI = MSG_config_PI(d_cfgTraj['Kp'], d_cfgTraj['Ki'])
        
    #generation d'un message de configuration des parametres du profil de vitesse
    msg_parametersT = MSG_config_profilVitesse(d_cfgTraj['TempsAcc'], d_cfgTraj['Facteur_vitesse_angulaire'], d_cfgTraj['Umax'])
    
    #generation d'un message d'init des parametres du modele des moteurs
    msg_parametersMotor = MSG_config_modeleMoteurCC(d_cfgTraj['Masse'],
                                                    d_cfgTraj['Rayon_roue'],
                                                    d_cfgTraj['Frottement_fluide'],
                                                    d_cfgTraj['Force_resistante'],
                                                    d_cfgTraj['Resistance_induit'],
                                                    d_cfgTraj['Inductance_induit'],
                                                    d_cfgTraj['Constance_couple'],
                                                    d_cfgTraj['Constante_vitesse'],
                                                    d_cfgTraj['Rapport_reduction'])
    
    #generation d'un message d'execution du test PI
    cmdTestPI = commandMsg("MSG_TEST_PI 0 0")
    msgTestPI = cmdTestPI.cmdMsgGeneration()
    
    #lancement du simulateur de deplacement
    print("Lancement du simulateur")
    simulator_process = subprocess.Popen('./simulator_trajAsser', shell=True, stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.PIPE)
    
    #initilisation des parametres de l'asser de trajectoire
    simulator_process.stdin.write(msg_parametersPI)
    simulator_process.stdin.write(msg_parametersT)
    simulator_process.stdin.write(msg_parametersMotor)
    
    #transmission de commandes de deplacement par l'entree standard
    simulator_process.stdin.write(msgTestPI)
    
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
    print("duree: " + str(d_traj["time"][0]) + "s")
    
    nbMaxPeriode = 0
    for log_list in d_traj.values() :
        if len(log_list) > nbMaxPeriode :
            nbMaxPeriode = len(log_list)
    periode = d_traj['periode'][0]
    duree = nbMaxPeriode * periode
    print[nbMaxPeriode, periode, str(nbMaxPeriode * d_traj['periode'][0]) + "s"]
    #~ sys.exit(2)
	
	#nombre de variable loguer
    print("Nombre de variable loguer: " + str(len(d_traj.keys())))
    #~ print("CPTvitesseAnglePtArret: " + str(d_traj["CPTvitesseAnglePtArret"]))
    #~ print("vitesseAnglePtArret: " + str(d_traj["vitesseAnglePtArret"]))
    if (d_traj.keys().count('finAsser') == 1): 
        print("Test fin asser OK")
        print(d_traj['finAsser'])
    else : print("Test fin asser negatif")
    
    print(len(d_traj["xRoueDroite"]))
    print(len(d_traj["xRoueGauche"]))
    
    figure(1)
    plot(d_traj["xRoueGauche"], d_traj["yRoueGauche"])
    hold(True)
    plot(d_traj["xRoueDroite"], d_traj["yRoueDroite"])
    plot(d_traj["xPoseReferenceRobot"], d_traj["yPoseReferenceRobot"], '-or')
    xCentre = [(d_traj["xRoueGauche"][index] + d_traj["xRoueDroite"][index])/2.0 for index in range(len(d_traj["xRoueDroite"]))]
    yCentre = [(d_traj["yRoueGauche"][index] + d_traj["yRoueDroite"][index])/2.0 for index in range(len(d_traj["yRoueDroite"]))]
    plot(xCentre, yCentre, '-m')
    posIndex = 70
    
    #~ plot((d_traj["xRoueGauche"][posIndex]+d_traj["xRoueDroite"][posIndex])/2.0, (d_traj["yRoueGauche"][posIndex]+d_traj["yRoueDroite"][posIndex])/2.0, 'ro')
    grid(True)
    title("trajectoire")
    
    figure(2)
    plot([index*periode for index in range(len(d_traj["vitesseMoteurGauche"]))], d_traj["vitesseMoteurGauche"], '-', label='mesure gauche')
    hold(True)
    plot([index*periode for index in range(len(d_traj["vitesseMoteurDroit"]))], d_traj["vitesseMoteurDroit"], label='mesure droit')
    #~ plot([index*periode for index in range(len(d_traj["ConsigneMoteurDroit"]))], [(cons-1024.0)*0.000985 for cons in d_traj["ConsigneMoteurDroit"]], label='consigne droit')
    #~ plot([index*periode for index in range(len(d_traj["vitesseProfilConsigne"]))], d_traj["vitesseProfilConsigne"], 'o', label='vitesse profil')
    #~ plot([index*periode for index in range(len(d_traj["vitLongitudinale"]))], d_traj["vitLongitudinale"], label='consigne long calculee')
    #~ plot([index*periode for index in range(len(d_traj["parametrePositionSegmentTrajectoire"]))], d_traj["parametrePositionSegmentTrajectoire"], label='paramTraj')
    #~ plot([index*periode for index in range(len(d_traj["xCCourant"]))], d_traj["xCCourant"], 'o', label='xCCourant')
    #~ plot([index*periode for index in range(len(d_traj["xPoseReferenceRobot"]))], d_traj["xPoseReferenceRobot"], 'o', label='xPoseReferenceRobot')
    #~ plot([index*periode for index in range(len(d_traj["distNormaliseeRestante"]))], d_traj["distNormaliseeRestante"], 'o', label='distNormaliseeRestante')
    #~ plot([index*periode for index in range(len(d_traj["distance_restante"]))], d_traj["distance_restante"], 'o', label='distance_restante')
    #~ plot([index*periode for index in range(len(d_traj["etat"]))], d_traj["etat"], label='etat')
    #~ plot([index*periode for index in range(len(d_traj["limit"]))], d_traj["limit"], 'om', label='limit')
    #~ plot([index*periode for index in range(len(d_traj["distance_decceleration"]))], d_traj["distance_decceleration"], '-', label='distDecc')
    
    #~ plot([index*periode for index in range(len(d_traj["ASSER_Running"]))], d_traj["ASSER_Running"], 'o', label='ASSER_Running')
    
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
    legend(loc="lower right")
    grid(True)
    title("vitesses")
    
    figure(3)
    plot([index*periode for index in range(len(d_traj["tensionPWM_MoteurGauche"]))], d_traj["tensionPWM_MoteurGauche"], label='moteur gauche')
    hold(True)
    plot([index*periode for index in range(len(d_traj["tensionPWM_MoteurDroit"]))], d_traj["tensionPWM_MoteurDroit"], label='moteur droit')
    legend()
    grid(True)
    title("tensions moteurs")
    
    figure(5)
    plot(d_traj["index_get_vitesseGabarit"])
    #~ plot(d_traj["index_tab_vit"])
    #~ plot(d_traj["dist_parcourue"], d_traj["val_tab_vit"])
    #~ plot(d_traj["val_tab_vit"])
    #~ plot(d_traj["dist_parcourue"])
    #~ plot(d_traj["angleRobot"], '-o')
    #~ plot(d_traj["angleRef"], '-o')
    #~ plot(d_traj["nbdepas"])
    #~ plot(d_traj["poseRefX"], 'o', label="poseRefX")
    #~ plot(d_traj["poseRobotX"], 'x', label="poseRobotX")
    #~ plot(d_traj["poseRefY"], 'o', label="poseRefY")
    #~ plot(d_traj["poseRobotY"], label="poseRobotY")
    print("poseRobotInitX: " + str(d_traj["poseRobotInitX"]))
    print("poseRobotInitY: " + str(d_traj["poseRobotInitY"]))
    #~ plot(d_traj["thetaPoseReference"], label="thetaPoseReference")
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
    legend(loc="lower right")
    
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
    pas = 0.005
    print("pas ech: " + str(pas))
    plot([index * pas for index in range(len(d_traj["gabarit_vitesse"]))], d_traj["gabarit_vitesse"], '-o', label='vit')
    
    grid()
    legend()
    
    
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


d_cfgTraj = {'Masse': 1.2
                , 'Rayon_roue': 0.03
                , 'Frottement_fluide': 0.0000504
                , 'Force_resistante': 0.4
                , 'Resistance_induit': 2.18
                , 'Inductance_induit': 0.00024
                , 'Constance_couple': 0.0234
                , 'Constante_vitesse': 0.02346
                , 'Rapport_reduction': 20.0
                , 'Kp': 8.0 #1.3 #1.49
                , 'Ki': 25.0 #3.2 #4.48
                , 'K1': 20.0
                , 'K2': 50.0
                , 'K3': 20.0
                , 'R1': -6.0
                , 'R2': -6.0
                , 'TempsAcc': 0.41	# tempsAcc
                , 'Facteur_vitesse_angulaire': 1.2	# facteur de vitesse angulaire
                , 'Umax': 900.0	# Umax
                , 'Distance': 0.3
                }

#########################################################################
#~ affichageTestPI2012(testPI(d_cfgTraj))
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
affichageGabaritVitesse_2012(traj)
#~ show()
affichageTraj2011(traj)


sys.exit(2)
