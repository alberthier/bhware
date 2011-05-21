#!/usr/bin/python

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
            
    return d_traj
    
#def trajFunction(K1 = 20.0, K2 = 75.0, K3 = 50.0, R1 = -10.2, R2 = -10.2, T1 = 3.0, T2 = 4.0, T3 = 900.0):
def trajFunction(Kp = 10.0, Ki = 5.0, K1 = 20.0, K2 = 75.0, K3 = 50.0, R1 = -10.2, R2 = -10.2, T1=2.0, T2=4.48, T3=900.0):
    #generation d'un message d'init de pose
    poseInit = commandMsg("INIT_POSE_ROBOT 0 0")
    #~ poseInit.addPose("0.0 0.0 0.0 0.0")
    poseInit.addPose("0.0 0.0 0.0 0.0")
    initPoseMessage = poseInit.cmdMsgGeneration()
    
    #generation d'un message d'init des parametres K
    parametersPI = commandMsg("PARAMETERS_PI")
    parametersPI.addPose("KP " + str(Kp))
    parametersPI.addPose("KI " + str(Ki))
    msg_parametersPI = parametersPI.cmdMsgGeneration()
    
    #generation d'un message d'init des parametres K
    parametersK = commandMsg("PARAMETERS_GAIN_K")
    parametersK.addPose("GAIN_K1 " + str(K1))
    parametersK.addPose("GAIN_K2 " + str(K2))
    parametersK.addPose("GAIN_K3 " + str(K3))
    msg_parametersK = parametersK.cmdMsgGeneration()
    
    #generation d'un message d'init des parametres pour la rotation
    parametersR = commandMsg("PARAMETERS_GAIN_ROT")
    parametersR.addPose("GAIN_R1 " + str(R1))
    parametersR.addPose("GAIN_R2 " + str(R2))
    msg_parametersR = parametersR.cmdMsgGeneration()
        
    #generation d'un message d'init des parametres du profil de vitesse
    parametersT = commandMsg("PARAMETERS_TIME")
    parametersT.addPose("ACC " + str(T1))
    parametersT.addPose("VITANGMAX " + str(T2))
    parametersT.addPose("UMAX " + str(T3))
    #print "UMAX " + str(T3)
    msg_parametersT = parametersT.cmdMsgGeneration()
    
    #generation d'un message de commande de deplacement #-1.57079632679
    dep1 = commandMsg("MSG_MAIN_GOTO 1 1")   # 'DEPLACEMENT' en 'MARCHE_AVANT'
    
    ##ligne droite
    dep1.addPose("2.55431082917 0.588033733948 2.144930857241 1")
    dep1.addPose("2.59863939238 0.688439898731 0.1651486774151 1")
    dep1.addPose("2.8 0.722 0.1651486774151 1")
    #demi-huit
    #~ dep1.addPose("0.7 0.0 0.0 1")
    #~ dep1.addPose("1.0 0.15 1.2 1")
    #~ dep1.addPose("0.95 0.35 2.6 1") 
    #~ dep1.addPose("0.6 0.35 -2.618 1")
    #~ dep1.addPose("0.0 0.0 -2.618 1")
    
    
    #dep1.addPose("-0.6 -0.35 -2.618 1")
    #dep1.addPose("-0.95 -0.35 2.6 1") 
    #dep1.addPose("-1.0 -0.15 1.2 1")
    #dep1.addPose("-0.7 0.0 0.0 1")
    #dep1.addPose("0.0 0.0 0.0 1")
    
    
    #dep1.addPose("1.0 0.0 0.0 1")
    #dep1.addPose("1.5 0.1 -0.1 1")
    #dep1.addPose("2.0 1.0 0.0 1")
    #dep1.addPose("2.5 1.0 0.0 1")
    
    #dep1.addPose("1.0 0.5 1.5 1.0")
    #dep1.addPose("2.0 1.0 0.0 1.0")
    
    depMessage1 = dep1.cmdMsgGeneration()
    
    depR = commandMsg("MSG_MAIN_GOTO 0 1")   # 'ROTATION' en 'MARCHE_AVANT'
    depR.addPose("-1.0 -0.2 0.0")
    depMessageR = depR.cmdMsgGeneration()
	
    depPI = commandMsg("MSG_MAIN_GOTO 1 1")   # 'DEPLACEMENT' en 'MARCHE_AVANT'
    depPI.addPose("2.0 0.0 0.0")
    depMessagePI = depPI.cmdMsgGeneration()
    
    #generation d'un message de commande de deplacement
    dep2 = commandMsg("MSG_MAIN_GOTO 1 1")   # 'DEPLACEMENT' en 'MARCHE_AVANT'
    dep2.addPose("2.0 -0.7 0.0 1")
    depMessage2 = dep2.cmdMsgGeneration()
    
    #lancement du simulateur de deplacement
    simulator_process = subprocess.Popen('./simulator_trajAsser', shell=True, stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.PIPE)
    
    #initilisation des parametres de l'asser de trajectoire
    simulator_process.stdin.write(msg_parametersPI)
    simulator_process.stdin.write(msg_parametersK)
    simulator_process.stdin.write(msg_parametersR)
    simulator_process.stdin.write(msg_parametersT)
    
    #transmission de l'init de pose par l'entree standard
    simulator_process.stdin.write(initPoseMessage)
    
    #transmission de commandes de deplacement par l'entree standard
    print("Envoi du message,")
    #~ simulator_process.stdin.write(depMessage1)
    #~ simulator_process.stdin.write(depMessageR)
    #~ simulator_process.stdin.write(depMessagePI)
    simulator_process.stdin.write(depMessage2)
    
    print("Fin du message.")
    #transmission de la commande de d'arret du simulateur
    (stdoutdata, stderrdata) = simulator_process.communicate("QUIT\n")
    
    #print 'len stdout:', len(stdoutdata)
    print("Simulation terminee.")
    #traitement des donnees de stdout
    lines = lineForm(stdoutdata)
    d_traj = stdoutParser(lines)
    #print("UMAX: " + str(d_traj["UMAX"][0]))
    
    return d_traj

#print len(d_traj.keys())
#for keyword in d_traj.keys():
#    print keyword

#sys.exit(2)

#figure(1)
#plot(d_traj["xRoueGauche"], d_traj["yRoueGauche"])
#hold(True)
#plot(d_traj["xRoueDroite"], d_traj["yRoueDroite"])
#grid(True)

#t = numpy.arange(0.0, len(d_traj["tensionPWM_MoteurGauche"])*d_traj["periode"][0], d_traj["periode"][0])
#figure(2)
#plot(t, d_traj["tensionPWM_MoteurGauche"])
#hold(True)
#plot(t, d_traj["tensionPWM_MoteurDroit"])
#plot(t, [d_traj["vitesseAngulaire"][i]*1000.0 for i in range(len(d_traj["vitesseAngulaire"]))])
#grid(True)
#show()

def testPI(Kp = 3000.0, Ki = 8000.0, T1=1.5, T2=4.48, T3=900.0):
    
    #generation d'un message d'init des parametres K
    parametersPI = commandMsg("PARAMETERS_PI")
    parametersPI.addPose("KP " + str(Kp))
    parametersPI.addPose("KI " + str(Ki))
    msg_parametersPI = parametersPI.cmdMsgGeneration()
        
    #generation d'un message d'init des parametres du profil de vitesse
    parametersT = commandMsg("PARAMETERS_TIME")
    parametersT.addPose("ACC " + str(T1))
    parametersT.addPose("VITANGMAX " + str(T2))
    parametersT.addPose("UMAX " + str(T3))
    #print "UMAX " + str(T3)
    msg_parametersT = parametersT.cmdMsgGeneration()
    
    #generation d'un message d'execution du test PI
    cmdTestPI = commandMsg("MSG_TEST_PI 0 0")
    msgTestPI = cmdTestPI.cmdMsgGeneration()
    
    #lancement du simulateur de deplacement
    simulator_process = subprocess.Popen('./simulator_trajAsser', shell=True, stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.PIPE)
    
    #initilisation des parametres de l'asser de trajectoire
    simulator_process.stdin.write(msg_parametersPI)
    simulator_process.stdin.write(msg_parametersT)
    
    #transmission de commandes de deplacement par l'entree standard
    #simulator_process.stdin.write(depMessage1)
    simulator_process.stdin.write(msgTestPI)
    
    #transmission de la commande de d'arret du simulateur
    (stdoutdata, stderrdata) = simulator_process.communicate("QUIT\n")
    
    #print 'len stdout:', len(stdoutdata)
    
    #traitement des donnees de stdout
    lines = lineForm(stdoutdata)
    d_traj = stdoutParser(lines)
    #print("UMAX: " + str(d_traj["UMAX"][0]))
    
    return d_traj

def trajFunctionT13(T13):
    d_traj = trajFunction(T1=T13[0], T3=T13[1])
    
    duree = d_traj["time"][0]
    if duree > (d_traj["periode"][0] / 2.0):
        nbPeriodeSaturation = 0.0
        nbPeriode = 0.001
        for valSat in d_traj["saturationPIgauche"]:
            if valSat > 0.5:
                nbPeriodeSaturation = nbPeriodeSaturation + 1
            nbPeriode = nbPeriode + 1
        tauxSat = (nbPeriodeSaturation / nbPeriode) * 100.0
    else:
        tauxSat = 0.0
    
    if duree < 0.0:
        duree = 1.0e6
    print [duree, tauxSat, T13[0], T13[1]]
    return (duree + 1000.0 * tauxSat)
    

def trajFunctionT123(T123):
    d_traj = trajFunction(T1=T123[0], T2=T123[1], T3=T123[2])
    
    duree = d_traj["time"][0]
    if duree > (d_traj["periode"][0] / 2.0):
        nbPeriodeSaturation = 0.0
        nbPeriode = 0.001
        for valSat in d_traj["saturationPIgauche"]:
            if valSat > 0.5:
                nbPeriodeSaturation = nbPeriodeSaturation + 1
            nbPeriode = nbPeriode + 1
        tauxSat = (nbPeriodeSaturation / nbPeriode) * 100.0
    else:
        tauxSat = 0.0
    
    if duree < 0.0:
        duree = 1.0e6
    print [duree, tauxSat, T123[0], T123[1], T123[2]]
    return (duree + 1000.0 * tauxSat)
    
def optimParamPI(PI):
    d_traj = testPI(Kp = PI[0], Ki = PI[1], T1 = 1.5, T3 = 900.0)
    
    sommeErreurVitesseCarre = 0.0
    for erreurVitesse in d_traj["erreurVitesseMoteurGauche"]:
        if erreurVitesse < 0.0:
            sommeErreurVitesseCarre = sommeErreurVitesseCarre + erreurVitesse * erreurVitesse
        else:
            sommeErreurVitesseCarre = sommeErreurVitesseCarre + erreurVitesse * erreurVitesse
    erreurTotale = math.sqrt(sommeErreurVitesseCarre / len(d_traj["erreurVitesseMoteurGauche"]))
    cout = erreurTotale
    
    #~ if duree < 0.0:
        #~ cout = 1.0e6
    print [cout, PI[0], PI[1]]
    return cout

def optimParamPI2(PI):
    d_traj = testPI(Kp = PI[0], Ki = PI[1], T1 = 1.0, T3 = 900.0)
    
    sommeErreurVitesseCarre = 0.0
    erreurMax = 0.0
    for erreurVitesse in d_traj["erreurVitesseMoteurGauche"]:
        if( math.fabs(erreurVitesse) > erreurMax) :
            erreurMax = math.fabs(erreurVitesse)
            
        if erreurVitesse < 0.0:
            sommeErreurVitesseCarre = sommeErreurVitesseCarre + erreurVitesse * erreurVitesse
        else:
            sommeErreurVitesseCarre = sommeErreurVitesseCarre + erreurVitesse * erreurVitesse
    erreurTotale = math.sqrt(sommeErreurVitesseCarre / len(d_traj["erreurVitesseMoteurGauche"]))
    cout = erreurTotale
    
    #~ if duree < 0.0:
        #~ cout = 1.0e6
    print [cout, PI[0], PI[1]]
    return erreurTotale, erreurMax

def optimParamPI_2011(PI):
    d_traj = testPI(Kp = PI[0], Ki = PI[1], T1 = 1.0, T3 = 900.0)
    
    sommeErreurVitesseCarre = 0.0
    tensionMax = 0.0
    # cout de la tension maximum
    for tension in d_traj["tensionPWM_MoteurGauche"]:
        if (tension > tensionMax) :
            tensionMax = tension
    coutTensionMax = math.fabs( ((1023-20) - tensionMax) / (1023-20))
            
    # cout de l'erreur de vitesse en regime permanent
    coutErreurVitesse = math.fabs(d_traj["erreurVitesseMoteurGauche"][-1] / d_traj["vitesseMoteurGauche"][-1])
        
    coutTotal = coutTensionMax + coutErreurVitesse
    
    #~ if duree < 0.0:
        #~ cout = 1.0e6
    print [coutTotal, PI[0], PI[1]]
    return coutTotal

def optimParamKi(l_Ki):
    d_traj = testPI(Kp = 5.7, Ki = l_Ki, T1 = 1.5, T3 = 900.0)
    
    sommeErreurVitesseCarre = 0.0
    for erreurVitesse in d_traj["erreurVitesseMoteurGauche"]:
        if erreurVitesse < 0.0:
            sommeErreurVitesseCarre = sommeErreurVitesseCarre + erreurVitesse * erreurVitesse
        else:
            sommeErreurVitesseCarre = sommeErreurVitesseCarre + erreurVitesse * erreurVitesse
    erreurTotale = math.sqrt(sommeErreurVitesseCarre / len(d_traj["erreurVitesseMoteurGauche"]))
    cout = erreurTotale
    
    #~ if duree < 0.0:
        #~ cout = 1.0e6
    print [cout, l_Ki]
    return (cout)

def afficheResLimitVitesse(d_traj):    
    figure(5)
    tempsNbP = [ periodeHN * x for x in range(len(d_traj["nbPeriode"]))]
    plot(tempsNbP, d_traj["nbPeriode"], 'ob')
    hold(True)
    plot(tempsNbP, d_traj["etat"], 'or')
    tempsA = [ periodeHN*10 * x for x in range(len(d_traj["vitLimite"]))]
    #~ plot(tempsA, d_traj["vitLimite"], label="vitLimite")
    #~ plot(tempsA, d_traj["vitFuturNormCC"], label="vitFuturNormCC")
    print("len vitLimite: " + str(len(d_traj["vitLimite"])))
    legend(loc="lower center")
	
    figure(6)
    #~ plot(d_traj["positionFutur"], d_traj["vitLimite"], 'o', label="vitLimite")
    #~ hold(True)
    #~ plot(d_traj["positionFutur"], d_traj["vitFuturNormCC"], 'o', label="vitFuturNormCC")
    #~ plot(d_traj["positionFutur"], [x*0.0002 for x in d_traj["nbP"]] , 'o', label="nbP")
    #~ for pos in d_traj["positionFutur"]:
        #~ plot([pos, pos], [-0.001, 0.005],'-k')
        
    #~ plot(d_traj["tempsFutur"], [x*1 for x in d_traj["futurVitesse"]] , '-ok', label="p")
        
    plot(d_traj["tempsFutur"], d_traj["vitLimite"], '-ob', linewidth=1, label="vitLimite")
    hold(True)
    plot(d_traj["tempsFutur"], d_traj["vitFuturNormCC"], '-og', linewidth=1, label="vitFuturNormCC")
    plot(d_traj["tempsFutur"], [x*0.0002 for x in d_traj["nbP"]] , '-or', linewidth=2, label="nbP")
    plot(d_traj["tempsFutur"], [x*0.0002 for x in d_traj["etatFutur"]] , '-oc', label="etatFutur")
    #plot(d_traj["tempsFutur"], [x*0.0001 for x in d_traj["futurVitesse"]] , '-ok', label="p")
    
    #~ for pos in d_traj["tempsFutur"]:
        #~ plot([pos, pos], [0.002, 0.008],'-k')
    #~ legend(loc="lower center")
    grid(True)
    v = axis()
    limits = []
    for val in v:
        limits.append(val)
    limits[2] = -0.0002
    limits[3] = 0.006
    axis(limits)
	

def affichageTraj(d_traj):
    #~ duree = d_traj["time"][0]
    
    nbMaxPeriode = 0
    for log_list in d_traj.values() :
        if len(log_list) > nbMaxPeriode :
            nbMaxPeriode = len(log_list)
    duree = nbMaxPeriode * d_traj['periode'][0]
    print[nbMaxPeriode, d_traj['periode'][0], str(nbMaxPeriode * d_traj['periode'][0]) + "s"]
    #~ sys.exit(2)
    
    if duree > (d_traj["periode"][0] / 2.0):
        nbPeriodeSaturation = 0.0
        nbPeriode = 0.001
        for valSat in d_traj["saturationPIgauche"]:
            if valSat > 0.5:
                nbPeriodeSaturation = nbPeriodeSaturation + 1
            nbPeriode = nbPeriode + 1
        tauxSat = (nbPeriodeSaturation / nbPeriode) * 100.0
    else:
        tauxSat = 0.0
    
    if duree < 0.0:
        duree = 1.0e6
    print([duree, tauxSat])
	
	#nombre de variable loguer
    print("Nombre de variable loguer: " + str(len(d_traj.keys())))
    
    figure(1)
    plot(d_traj["xRoueGauche"], d_traj["yRoueGauche"])
    hold(True)
    plot(d_traj["xRoueDroite"], d_traj["yRoueDroite"])
    #~ plot(d_traj["xPoseReferenceRobot"], d_traj["yPoseReferenceRobot"])
    grid(True)
    title("trajectoire")
    
    #t = numpy.arange(0.0, (len(d_traj["parametrePositionSegmentTrajectoire"]))*d_traj["periode"][0], d_traj["periode"][0])
    #t2 = numpy.arange(0.0, len(d_traj["xCCourant"])*d_traj["periode"][0], d_traj["periode"][0])
    #print(len(t))
    #print(len(d_traj["parametrePositionSegmentTrajectoire"]))
    #print(len(t2))
    #print(len(d_traj["xCCourant"]))
    #figure(2)
    #plot(t, d_traj["parametrePositionSegmentTrajectoire"])
    ##plot(t, d_traj["tensionPWM_MoteurGauche"])
    ##plot(t, d_traj["vitesseMoteurGauche"])
    #hold(True)
    ##plot(t, d_traj["xPoseReferenceRobot"])
    #plot(t2, d_traj["xCCourant"])
    #plot(t2, d_traj["yCCourant"])
    ##plot(t, d_traj["tensionPWM_MoteurGauche"])
    ##plot(t, d_traj["vitesseNormaliseeCourante"])
    ##plot(t, d_traj["vitesseMoteurDroit"])
    ###plot(t, [d_traj["vitesseAngulaire"][i]*1000.0 for i in range(len(d_traj["vitesseAngulaire"]))])
    #grid(True)
    
    periodeHN = d_traj["periode"][0]
    periodeBN = 0.001   #1ms
	
    tempsU = [ periodeHN * x for x in range(len(d_traj["tensionPWM_MoteurGauche"]))]
    
    figure(2)
    plot(tempsU, d_traj["tensionPWM_MoteurGauche"], label="tension moteur gauche")
    hold(True)
    #~ plot([ periodeBN * x for x in range(len(d_traj["vitesseG1ms"]))], d_traj["vitesseG1ms"], label="tension gauche 1ms")
    plot(tempsU, d_traj["tensionPWM_MoteurDroit"], label="tension moteur droit")
    #~ plot([ periodeBN * x for x in range(len(d_traj["vitesseD1ms"]))], d_traj["vitesseD1ms"], 'o', markersize=4, label="tension droite 1ms")
    plot(tempsU, d_traj["integPIVG"], label="integG")
    plot(tempsU, d_traj["integPIVD"], label="integD")
    grid(True)
    title("tensions moteurs")
    legend(loc="lower center")
    v = axis()
    limits = []
    for val in v:
        limits.append(val)
    limits[1] = 0.03
    limits[3] = 200
    #~ axis(limits)
    
    temps = numpy.arange(0.0, (len(d_traj["vitesseMoteurGauche"]))*d_traj["periode"][0], d_traj["periode"][0])
    tempsVitNorm = numpy.arange(0.0, (len(d_traj["vitesseNormaliseeCourante"]))*d_traj["periode"][0], d_traj["periode"][0])
    print(len(d_traj["ConsigneMoteurDroit"]))
    #print(d_traj["xDerivPos"])
    tempsV = [ periodeHN * x for x in range(len(d_traj["vitesseMoteurGauche"]))]
    figure(3)
    plot(tempsV, d_traj["vitesseMoteurGauche"], label="moteur gauche")
    hold(True)
    #~ plot([ periodeBN * x for x in range(len(d_traj["vitesseG1ms"]))], d_traj["vitesseG1ms"], label="vitesse gauche 1ms")
    plot(tempsV, d_traj["vitesseMoteurDroit"], label="moteur droit")
    #~ plot([ periodeBN * x for x in range(len(d_traj["vitesseD1ms"]))], d_traj["vitesseD1ms"], 'o', markersize=4, label="vitesse droite 1ms")
    plot(tempsV, d_traj["vitesseNormaliseeCourante"],label="vit norm")
    #plot(tempsV, [(d_traj["ConsigneMoteurDroit"][index]-1023)*0.00083 for index in range(len(d_traj["ConsigneMoteurDroit"]))], label="vit cons droit")
    #plot(tempsV, [(d_traj["ConsigneMoteurGauche"][index]-1023)*0.00083 for index in range(len(d_traj["ConsigneMoteurGauche"]))], label="vit cons gauche")
    #plot(temps, d_traj["ConsigneMoteurDroit"], label="vit cons droit")
    #plot(temps, d_traj["ConsigneMoteurGauche"], label="vit cons gauche")
    plot(tempsV, d_traj["erreurVitesseMoteurDroit"], label="err vit droit")
    #plot(temps, [(d_traj["ConsigneMoteurGauche"][index]-103.0)*0.00089 for index in range(len(d_traj["ConsigneMoteurGauche"]))])
    #plot(temps, [(d_traj["ConsigneMoteurDroit"][index]-1023.0)*0.00089 for index in range(len(d_traj["ConsigneMoteurDroit"]))])
    #~ tempsVit = [ periodeHN * x for x in range(len(d_traj["vitLongitudinale"]))]
    tempsVitR = [ periodeHN * x for x in range(len(d_traj["vitRotation"]))]
    plot(tempsVitR, d_traj["vitLongitudinale"], label="vitLong")
    #~ plot(tempsVitR, d_traj["vitRotation"], label="vitRot")
    #~ tempsVN = [ periodeHN + periodeHN * x for x in range(len(d_traj["vitnormInitiale"]))]
    #~ plot(tempsVN, d_traj["vitnormInitiale"], label="vitnormI")
    
    #plot(temps, [math.sqrt(d_traj["xDiffTemporellePoseRefRobot"][index]*d_traj["xDiffTemporellePoseRefRobot"][index] + d_traj["yDiffTemporellePoseRefRobot"][index]*d_traj["yDiffTemporellePoseRefRobot"][index]) for index in range(len(d_traj["xDiffTemporellePoseRefRobot"]))], '-c' )
    title("Vitesses")
    legend(loc="upper right")
    grid(True)
    
    v = axis()
    limits = []
    for val in v:
        limits.append(val)
    limits[2] = -1.5
    limits[3] = 3.5
    axis(limits)
    
    #~ figure(4)
    #~ tempsPPST = [ periodeHN * x for x in range(len(d_traj["parametrePositionSegmentTrajectoire"]))]
    #~ plot(tempsPPST, d_traj["parametrePositionSegmentTrajectoire"], label="position")
    #~ hold(True)
    #~ #plot(tempsPPST, d_traj["distNormaliseeRestante"], label="dist restante")
    #~ #plot([(d_traj["xErreurPoseCentreRobot"][index]/math.fabs(d_traj["xErreurPoseCentreRobot"][index]))*math.sqrt(d_traj["xErreurPoseCentreRobot"][index]*d_traj["xErreurPoseCentreRobot"][index] + d_traj["yErreurPoseCentreRobot"][index]*d_traj["yErreurPoseCentreRobot"][index]) for index in range(len(d_traj["xErreurPoseCentreRobot"]))])
    #~ tempsErrP = [ periodeHN * x for x in range(len(d_traj["xErreur_P"]))]
    #~ #plot(tempsErrP, d_traj["xErreur_P"], label="erreur x")
    #~ #plot(tempsErrP, d_traj["yErreur_P"], label="erreur y")
    #~ 
    #~ tempsEtat = [ periodeHN * x for x in range(len(d_traj["etat"]))]
    #~ plot(tempsEtat, d_traj["etat"], 'or')
    #~ #plot(tempsEtat, d_traj["vitP"], 'o', label="vitP")
    #~ 
    #~ plot(tempsPPST, [x/50 for x in d_traj["diffTheta"]], label="courbure")
    #~ print("diffTheta: " + str(max(d_traj["diffTheta"])))
    #~ 
    #~ grid(True)
    #~ title("Parametre position")
    #~ legend(loc="lower right")
    #~ v = axis()
    #~ limits = []
    #~ for val in v:
        #~ limits.append(val)
    #~ #limits[2] = -0.2
    #~ #limits[3] = 0.1
    #~ axis(limits)

    #~ afficheResLimitVitesse(d_traj)
    #~ figure(7)
    #~ tempsP = [ periodeHN * x for x in range(len(d_traj["p"]))]
    #~ plot(tempsP, d_traj["p"],'o')
    #~ grid(True)

    #~ figure(8)
    #~ tempsDistMinDecc = [ periodeHN * x for x in d_traj["cptDecc"]]
    #~ plot(tempsDistMinDecc, d_traj["distMinDecc"], label="distMinDecc")
    #~ hold(True)
    #~ plot(tempsDistMinDecc, d_traj["distRm"], label="distRm")
    #~ grid(True)
    #~ legend(loc="center left")

    #~ figure(9)
    #~ tempsErrPose = [ periodeHN * x for x in range(len(d_traj["xErreurPoseCentreRobot"]))]
    #~ plot(tempsErrPose, d_traj["xErreurPoseCentreRobot"], label="xErr")
    #~ hold(True)
    #~ plot(tempsErrPose, d_traj["yErreurPoseCentreRobot"], label="yErr")
    #~ grid(True)
    #~ legend()

    #~ print("pmax: " + str(d_traj["pmax"]))
    #~ print("errDist: " + str(d_traj["errDist"][-1]))
    #~ 
    #~ print("distance: " + str(d_traj["distance"]))
    #~ 
    #~ #print("len_t: " + str(len(d_traj["tCourbeBS"])))
#~ 
    #~ if (d_traj.keys().count("delta2t") == 1):
        #~ print("delta2t: " + str(d_traj["delta2t"]))
    #~ if (d_traj.keys().count("delta2t_0") == 1):
        #~ print("delta2t_0: " + str(d_traj["delta2t_0"]))
    #~ if (d_traj.keys().count("delta2t_1") == 1):
        #~ print("delta2t_1: " + str(d_traj["delta2t_1"]))
        
    #~ for k in d_traj.keys():
        #~ print k
	
    #for message in d_traj["msgR"]:
        #print message
    #print(d_traj["data"])
    #print("t: " + str(len(t)) + ", lenVitM: " + str(len(d_traj["vitesseMoteurGauche"])))
    #print("x_traj: " + str(d_traj["x_traj"]))
    #print("y_traj: " + str(d_traj["y_traj"]))
    #print("angle_traj: " + str(d_traj["angle_traj"]))
    #print("mask_traj: " + str(d_traj["mask_traj"]))
    
    #print(d_traj["deltaI"])
    #print(d_traj["qI"])
    
    #print("deltaFx: " + str(d_traj["deltaFx"]))
    #print("deltaFy: " + str(d_traj["deltaFy"]))
    
    #print("tFinal: " + str(d_traj["tFinal"]))
    #print("xConsFinal: " + str(d_traj["xConsFinal"]))
    #print("xConsFinal_F: " + str(d_traj["xConsFinal_F"]))
    
    
    
    #print("ab: " + str(d_traj["ab"]))
    #print("iCGP0: " + str(d_traj["iCGP0"]))
    #print("iCGP1: " + str(d_traj["iCGP1"]))
    #print("g1pt: " + str(d_traj["g1pt"]))
    #t2 = numpy.arange(0.0, len(d_traj["tensionPWM_N1"])*d_traj["periode"][0]/2.0, d_traj["periode"][0]/2.0)
    #figure(3)
    #plot(t2, d_traj["tensionPWM_N1"])
    #grid(True)
    
    show()

def affichageTraj2011(d_traj):
    #~ duree = d_traj["time"][0]
    
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
    plot(d_traj["xPoseReferenceRobot"], d_traj["yPoseReferenceRobot"], '-r')
    xCentre = [(d_traj["xRoueGauche"][index] + d_traj["xRoueDroite"][index])/2.0 for index in range(len(d_traj["xRoueDroite"]))]
    yCentre = [(d_traj["yRoueGauche"][index] + d_traj["yRoueDroite"][index])/2.0 for index in range(len(d_traj["yRoueDroite"]))]
    plot(xCentre, yCentre, '-m')
    posIndex = 70
    
    #~ plot((d_traj["xRoueGauche"][posIndex]+d_traj["xRoueDroite"][posIndex])/2.0, (d_traj["yRoueGauche"][posIndex]+d_traj["yRoueDroite"][posIndex])/2.0, 'ro')
    grid(True)
    title("trajectoire")
    
    figure(2)
    plot([index*periode for index in range(len(d_traj["vitesseMoteurGauche"]))], d_traj["vitesseMoteurGauche"], 'o', label='mesure gauche')
    hold(True)
    plot([index*periode for index in range(len(d_traj["vitesseMoteurDroit"]))], d_traj["vitesseMoteurDroit"], label='mesure droit')
    plot([index*periode for index in range(len(d_traj["ConsigneMoteurDroit"]))], [(cons-1024.0)*0.000985 for cons in d_traj["ConsigneMoteurDroit"]], label='consigne droit')
    plot([index*periode for index in range(len(d_traj["vitesseProfilConsigne"]))], d_traj["vitesseProfilConsigne"], 'o', label='vitesse profil')
    plot([index*periode for index in range(len(d_traj["vitLongitudinale"]))], d_traj["vitLongitudinale"], label='consigne long calculee')
    #~ plot([index*periode for index in range(len(d_traj["parametrePositionSegmentTrajectoire"]))], d_traj["parametrePositionSegmentTrajectoire"], label='paramTraj')
    #~ plot([index*periode for index in range(len(d_traj["xCCourant"]))], d_traj["xCCourant"], 'o', label='xCCourant')
    #~ plot([index*periode for index in range(len(d_traj["xPoseReferenceRobot"]))], d_traj["xPoseReferenceRobot"], 'o', label='xPoseReferenceRobot')
    #~ plot([index*periode for index in range(len(d_traj["distNormaliseeRestante"]))], d_traj["distNormaliseeRestante"], 'o', label='distNormaliseeRestante')
    #~ plot([index*periode for index in range(len(d_traj["distance_restante"]))], d_traj["distance_restante"], 'o', label='distance_restante')
    #~ plot([index*periode for index in range(len(d_traj["etat"]))], d_traj["etat"], label='etat')
    #~ plot([index*periode for index in range(len(d_traj["limit"]))], d_traj["limit"], 'om', label='limit')
    #~ plot([index*periode for index in range(len(d_traj["distance_decceleration"]))], d_traj["distance_decceleration"], '-', label='distDecc')
    
    #~ plot([index*periode for index in range(len(d_traj["ASSER_Running"]))], d_traj["ASSER_Running"], 'o', label='ASSER_Running')
    
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
    plot(d_traj["index_tab_vit"])
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
    
    #~ print(d_traj["refX"][0])
    
    #~ affichageGabaritVitesse(d_traj)
    
    show()

def affichageTestPI(d_traj):
    #duree = d_traj["time"][0]
	
	#nombre de variable loguer
    #print("Nombre de variable loguer: " + str(len(d_traj.keys())))
    
    periode=d_traj["periode"][0]
    temps = [ periode * x for x in range(len(d_traj["vitesseMoteurGauche"]))]
    
    figure(2)
    plot(temps, d_traj["tensionPWM_MoteurGauche"], label="tension moteur gauche")
    hold(True)
    #~ plot(temps, d_traj["tensionPWM_MoteurDroit"], label="tension moteur droit")
    grid(True)
    legend(loc="lower center")
    title("tensions moteurs")
    
    #~ #temps = [temps[index] for index in range(len(temps)-1)]
    print("periode: %s" % periode)
    print("dernier temps: %s" % temps[-1]) 
    print("len_temps: " + str(len(temps)))
    print("len_vitMotG: " + str(len(d_traj["vitesseMoteurGauche"])))
    print("len_consMotG: " + str(len(d_traj["ConsigneMoteurGauche"])))
    print("len_erVitMotG: " + str(len(d_traj["erreurVitesseMoteurGauche"])))
    
    figure(3)
    plot(temps, [(d_traj["ConsigneMoteurGauche"][index]-1023)*0.000985 for index in range(len(d_traj["ConsigneMoteurGauche"]))], label="vit cons gauche")
    hold(True)
    plot(temps, [d_traj["vitesseMoteurGauche"][index] for index in range(len(d_traj["vitesseMoteurGauche"]))], label="vitesse moteur gauche")
    plot(temps, d_traj["erreurVitesseMoteurGauche"], label="err vit gauche")
    
    title("Vitesses")
    legend(loc="lower center")
    grid(True)
    
    v = axis()
    limits = []
    for val in v:
        limits.append(val)
    limits[2] = -1.5
    limits[3] = 1.5
    axis(limits)
    
    sommeErreurVitesseCarre = 0.0
    for erreurVitesse in d_traj["erreurVitesseMoteurGauche"]:
        if erreurVitesse < 0.0:
            sommeErreurVitesseCarre = sommeErreurVitesseCarre + erreurVitesse * erreurVitesse
        else:
            sommeErreurVitesseCarre = sommeErreurVitesseCarre + erreurVitesse * erreurVitesse
    erreurTotale = math.sqrt(sommeErreurVitesseCarre / len(d_traj["erreurVitesseMoteurGauche"]))
    cout = erreurTotale
    
    print [cout, d_traj["Ki"]]
    
    show()

def afficheCoutPlageKi(Kp):
    l_Ki = range(0, 50, 10)

    l_cout = [optimParamPI([Kp, k]) for k in l_Ki]
    figure(1)
    plot(l_Ki, l_cout)
    show()


#~ def theOptimizerDePI:
    #~ # les Kp
    #~ mKp={}
    #~ 
    #~ l_Kp = 
    #~ for k in range(0, 20, 0.2) :
        #~ mKp[k] = optimParamPI2([k, 0])[1]
    #~ 
    #~ #ici
#~ 
    #~ l_cout = map(lambda k :  for k in l_Kp ]
    #~ 
    #~ 
    #~ figure(1)
    #~ plot(l_Ki, l_cout)
    #~ show()
    
def ecrireCoutPlageKp():
    l_Kp = [x for x in range(60)]
    l_KpCout = []
    for Kp in l_Kp:
        tmp = optimParamPI2([Kp, 0])
        l_KpCout.append(str(Kp) + " " + str(tmp[0]) + " " + str(tmp[1]) + '\n')
    
    fic = open("plageCoutKpG", 'w')
    fic.writelines(l_KpCout)
    fic.close()

def ecrireCoutPlageKi(Kp):
    l_Ki = [x*2 for x in range(60)]
    l_KiCout = []
    for Ki in l_Ki:
        tmp = optimParamPI2([Kp, Ki])
        l_KiCout.append(str(Kp) + " " + str(Ki) + " " + str(tmp[0]) + " " + str(tmp[1]) + '\n')
    
    fic = open("plageCoutKiG", 'w')
    fic.writelines(l_KiCout)
    fic.close()
 
def plotCoutPlageKp():   
    fic = open("plageCoutKpG", 'r')
    l_KpCout = fic.readlines()
    fic.close()
    l_Kp = []
    l_Cout = [[], []]
    for KpCout in l_KpCout:
        tmp = KpCout.split()
        l_Kp.append(float(tmp[0]))
        l_Cout[0].append(float(tmp[1]))
        l_Cout[1].append(float(tmp[2]))
        
    figure(1)
    plot(l_Kp, l_Cout[0], label="erreur des moindres carres")
    hold(True)
    plot(l_Kp, l_Cout[1], label="erreur max")
    legend(loc="upper right")
    grid()
    show()
        
def plotCoutPlageKi():   
    fic = open("plageCoutKiG", 'r')
    l_KiCout = fic.readlines()
    fic.close()
    l_Ki = []
    l_Cout = [[], []]
    for KiCout in l_KiCout:
        tmp = KiCout.split()
        l_Ki.append(float(tmp[1]))
        l_Cout[0].append(float(tmp[2]))
        l_Cout[1].append(float(tmp[3]))
        
    figure(1)
    plot(l_Ki, l_Cout[0], label="erreur des moindres carres")
    hold(True)
    plot(l_Ki, l_Cout[1], label="erreur max")
    legend(loc="upper right")
    grid()
    show()

def affichageGabaritVitesse(d_traj):
        
    figure(4)
    # affichage avec un pas de 2cm
    pas = d_traj["pas_ech"][0]
    print("pas ech: " + str(pas))
    plot([index * pas for index in range(len(d_traj["gabarit_vitesse"]))], d_traj["gabarit_vitesse"], '-o', label='vit')
    plot([index * pas for index in range(len(d_traj["phase_acc"]))], d_traj["phase_acc"], '-o', label='phase')
    #~ plot([index * pas for index in range(len(d_traj["vit_ph1"]))], d_traj["vit_ph1"], '-', label='vit_ph1')
    print("len phase_acc: " + str(len(d_traj["phase_acc"])))
    plot([index * pas for index in range(len(d_traj["gabarit_acceleration"]))], d_traj["gabarit_acceleration"], '-o', label='acc')
    #~ plot([index * pas for index in range(len(d_traj["distanceRestante_FD"]))], d_traj["distanceRestante_FD"], '-', label='distRest_FD')
    #~ plot([index * pas for index in range(len(d_traj["distanceRestante_temp"]))], d_traj["distanceRestante_temp"], '-', label='distRest_temp')

    acc = [0]
    for index in range(len(d_traj["gabarit_vitesse"])-1) : 
        acc.append((math.pow(d_traj["gabarit_vitesse"][index+1],2.0) - math.pow(d_traj["gabarit_vitesse"][index], 2.0)) / (2.0*0.02))
    #~ acc.append(0)
    #~ plot([index * pas for index in range(len(acc))], [acc[index]*1 for index in range(len(acc))], '-o', label = 'acc')
    
    grid()
    legend()
    print("len gabarit: " + str(len(d_traj["gabarit_vitesse"])) + ", max: " + str(max(d_traj["gabarit_vitesse"])))
    #print(d_traj["gabarit_vitesse"][0:5])
    print("vmax: " + str(d_traj["vmax"]))
    print("acc_D1: " + str(d_traj["acc_D1"]))
    print("acc_D2: " + str(d_traj["acc_D2"]))
    #~ print("decc_D3: " + str(d_traj["decc_D3"]))
    #~ print("decc_D4: " + str(d_traj["decc_D4"]))
    #~ print("vit_ph1: " + str(d_traj["vit_ph1"]))
    #~ print("vit_ph1b: " + str(d_traj["vit_ph1b"]))
    #~ print("vit_ph12: " + str(d_traj["vit_ph12"]))
    #~ print("Acc_np1: " + str(d_traj["Acc_np1"]))
    #~ print("espAcc_np1: " + str(d_traj["espAcc_np1"]))
    tps_tot = 0
    for tps in d_traj["espAcc_np1"][:8] : #34
        tps_tot = tps_tot + tps
    print("temps total 1: " + str(tps_tot))
    
    

#affichageTraj(trajFunction(T1=4.54, T3=707.0)) #T1=1.57
#affichageTraj(trajFunction(T1=4.43, T2=4.13, T3=712.0))
#affichageTraj(trajFunction(T1=1.23, T2=4.48, T3=800.0))
#print trajFunctionT13([1.32, 859.0])

traj = trajFunction(Kp = 1.49
                    , Ki = 4.48
                    , K1 = 20.0
                    , K2 = 50.0
                    , K3 = 20.0
                    , R1 = -6.0
                    , R2 = -6.0
                    , T1 = 2.0	# tempsAcc
                    , T2 = 1.2	# facteur de vitesse angulaire
                    , T3 = 900.0	# Umax
                    )
affichageGabaritVitesse(traj)
#~ show()
affichageTraj2011(traj)


sys.exit(2)
#~ 
#~ affichageTestPI(testPI(Kp = 1.49 #14.0
                        #~ , Ki = 4.48 #27.0
                        #~ , T1=1.0 #1.5
                        #~ , T2=4.48
                        #~ , T3=900.0
                        #~ ))
#~ 
#~ # ecrireCoutPlageKi(1.0)
#~ # plotCoutPlageKi()
#~ sys.exit(2)

#~ #print(trajFunctionPI([3000.0, 8000]))
#~ #sys.exit(2)

##optimisation de T1 et T3
#p0 = [4.3, 750.0]
##solT13 = fmin_slsqp(trajFunctionT13, p0, bounds=[(0.1,6.0), (200.0, 1023.0)], iprint=2, full_output=1)
#solT13 = fmin(trajFunctionT13, p0, xtol=1.0, ftol=0.1, maxiter=45, disp = True, retall = True)
#print solT13[0]

##optimisation de T1, T2 et T3
#p0 = [4.3, 4.0, 750.0]
#solT123 = fmin(trajFunctionT123, p0, xtol=1.0, ftol=0.1, maxiter=45, disp = True, retall = True)
#print solT123[0]

#optimisation des gains PI
p0 = [2.0, 2.0]
solPI = fmin(optimParamPI_2011, p0, xtol=0.01, ftol=0.001, maxiter=45, disp = True, retall = True)
print solPI[0]
sys.exit(2)

#~ #optimisation de Ki
#~ p0 = [10.0]
#~ solKi = fmin(optimParamKi, p0, xtol=0.0001, ftol=0.0001, maxiter=45, disp = True, retall = True)
#~ print solKi[0]
