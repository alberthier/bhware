#!/usr/bin/python

import subprocess
import sys
import time
from matplotlib.pyplot import *
from pylab import *
import Image
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

class point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def listeDeplacements():
	logDep = open("matchlog.txt", 'r')
	lines = logDep.readlines()
	logDep.close()
	
	dep = []
	for line in lines:
		res = re.search('\"(\w+)\"', line)
		if (res != None):			
			dico = {"mvt":res.groups()[0]}
			res = re.findall('(\w+)\s*=\s*([\d.-]+)', line)
			if (res != None):
				for coupleKV in res:
					dico[coupleKV[0]] = float(coupleKV[1])
				#~ print dico
				dep.append(dico)
	
	#~ print(len(dep))
	return(dep)

def listeDeplacements():
	logDep = open("matchlog.txt", 'r')
	lines = logDep.readlines()
	logDep.close()
	
	dep = []
	for line in lines:
		res = re.search('\"(\w+)\"', line)
		if (res != None):			
			dico = {"mvt":res.groups()[0]}
			res = re.findall('(\w+)\s*=\s*([\d.-]+)', line)
			if (res != None):
				for coupleKV in res:
					dico[coupleKV[0]] = float(coupleKV[1])
				#~ print dico
				dep.append(dico)
	
	#~ print(len(dep))
	return(dep)
    
#def trajFunction(K1 = 20.0, K2 = 75.0, K3 = 50.0, R1 = -10.2, R2 = -10.2, T1 = 3.0, T2 = 4.0, T3 = 900.0):
def trajFunction(Kp = 10.0, Ki = 5.0, K1 = 20.0, K2 = 75.0, K3 = 50.0, R1 = -10.2, R2 = -10.2, T1=2.0, T2=4.48, T3=900.0):
    #generation d'un message d'init de pose
    poseInit = commandMsg("INIT_POSE_ROBOT 0 0")
    poseInit.addPose("2.815 0.185 2.14493085724 0.0")
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
    
    #lancement du simulateur de deplacement
    simulator_process = subprocess.Popen('./simulator_trajAsser', shell=True, stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.PIPE)
    
    #initilisation des parametres de l'asser de trajectoire
    simulator_process.stdin.write(msg_parametersPI)
    simulator_process.stdin.write(msg_parametersK)
    simulator_process.stdin.write(msg_parametersR)
    simulator_process.stdin.write(msg_parametersT)
    
    #transmission de l'init de pose par l'entree standard
    simulator_process.stdin.write(initPoseMessage)
    
    #Sequence de match
    l_dep = listeDeplacements()
    mouvement = "1"
    sens = "1"

    pF = point(2.815, 0.185)
    thetaF = math.atan2(0.672-pF.y, 2.5-pF.x)
    cpt_dep = 0
    l_ptSuppr = [1,2,3,6,7,10,11,14,15,19,20,23,24,27,28,34,35,38,39,42,43,46,47,50,51,54,55,58,59,62,63,66,67]
    l_ptSuppr = []
    
    for dep in l_dep[:2]:
        cpt_dep = cpt_dep + 1
        if (l_ptSuppr.count(cpt_dep) == 0):
            #~ print(dep)
            if (dep["mvt"] == 'move'):
                mouvement = "1"
                
                pI = pF
                pF = point(dep["x"]/1000.0, dep["y"]/1000.0)
                if (cpt_dep == 17) or (cpt_dep == 30):
                    thetaI = math.pi + thetaF
                    thetaF = math.pi + dep['angle']
                    sens = "-1"
                else:
                    thetaI = thetaF
                    thetaF = dep['angle']
                    sens = "1"
                
                #calcul du point intermediaire
                thetaP = math.atan2(pF.y-pI.y, pF.x-pI.x)
                
                theta10 = thetaP - thetaI
                if theta10 < 0:
                    theta10 = theta10 + 2*math.pi
                if (theta10 > math.pi):
                    theta10 = 2*math.pi - theta10
                    
                theta11 = thetaP - thetaF
                if theta11 < 0:
                    theta11 = theta11 + 2*math.pi
                if theta11 > math.pi:
                    theta11 = 2*math.pi - theta11
                    
                dist = math.sqrt(pow(pF.x-pI.x,2) + pow(pF.y-pI.y,2))
                d = dist*(math.sin(theta11)/math.sin(math.pi-theta10-theta11))
                pinter = point(pI.x + d*math.cos(thetaI), pI.y + d*math.sin(thetaI))
                
                B = 0.1
                p1 = point(pinter.x-B*cos(thetaI), pinter.y-B*sin(thetaI))
                p2 = point(pinter.x+B*cos(thetaF), pinter.y+B*sin(thetaF))
                
                deltaD = 0.1
                #generation d'un message de commande de deplacement
                dep1 = commandMsg("MSG_MAIN_GOTO " + mouvement + " " + sens)   # 'DEPLACEMENT' en 'MARCHE_AVANT'
                dep1.addPose(str(pF.x-deltaD*cos(thetaF)) + " " + str(pF.y-deltaD*sin(thetaF)) + " " + str(thetaF) + " 1")
                #~ dep1.addPose(str(p1.x) + " " + str(p1.y) + " " + str(thetaI) + " 1")
                #~ dep1.addPose(str(p2.x) + " " + str(p2.y) + " " + str(thetaF) + " 1")
                #~ dep1.addPose(str(pF.x) + " " + str(pF.y) + " " + str(thetaF) + " 1")
                print("Distance a parcourir: " + str(math.sqrt(pow(pF.x-deltaD*cos(thetaF)-pI.x,2) + pow(pF.y-deltaD*sin(thetaF)-pI.y,2))))
                
                
            else:
                mouvement = "0"
                sens = "1"
                thetaI = thetaF
                thetaF = math.atan2(dep["y"]/1000.0 - pF.y, dep["x"]/1000.0 - pF.x)
                #generation d'un message de commande de deplacement
                dep1 = commandMsg("MSG_MAIN_GOTO " + mouvement + " " + sens)   # 'DEPLACEMENT' en 'MARCHE_AVANT'
                dep1.addPose(str(dep["x"]/1000.0) + " " + str(dep["y"]/1000.0) + " " + str(thetaF) + " 1")
            
            #transmission de commandes de deplacement par l'entree standard
            depMessage1 = dep1.cmdMsgGeneration()
            print(depMessage1)
            simulator_process.stdin.write(depMessage1)
    
            
    #~ dep1 = commandMsg("MSG_MAIN_GOTO 1 1")   # 'DEPLACEMENT' en 'MARCHE_AVANT'
    #~ ##ligne droite
    #~ dep1.addPose("2.55431082917 0.588033733948 2.144930857241 1")
    #~ dep1.addPose("2.59863939238 0.688439898731 0.1651486774151 1")
    #~ dep1.addPose("2.8 0.722 0.1651486774151 1")
    #~ depMessage1 = dep1.cmdMsgGeneration()
    #~ simulator_process.stdin.write(depMessage1)
    
    print("Simulation en cours ...")
    #transmission de la commande de d'arret du simulateur
    (stdoutdata, stderrdata) = simulator_process.communicate("QUIT\n")
    #print 'len stdout:', len(stdoutdata)
    
    #traitement des donnees de stdout
    lines = lineForm(stdoutdata)
    d_traj = stdoutParser(lines)
    
    return d_traj


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
	
    duree = d_traj["periode"][0]
    #~ if duree > (d_traj["periode"][0] / 2.0):
        #~ nbPeriodeSaturation = 0.0
        #~ nbPeriode = 0.001
        #~ for valSat in d_traj["saturationPIgauche"]:
            #~ if valSat > 0.5:
                #~ nbPeriodeSaturation = nbPeriodeSaturation + 1
            #~ nbPeriode = nbPeriode + 1
        #~ tauxSat = (nbPeriodeSaturation / nbPeriode) * 100.0
    #~ else:
        #~ tauxSat = 0.0
    #~ 
    #~ if duree < 0.0:
        #~ duree = 1.0e6
    #~ print([duree, tauxSat])
    print("duree: " + str(duree*len(d_traj["xRoueGauche"])) + "s")
	
	#nombre de variable loguer
    print("Nombre de variable loguer: " + str(len(d_traj.keys())))
    
    figure(1)
    # affichage de l'image du terrain
    terrain = imread('map.png')
    extent = (-0.019, 3.098, -0.022, 2.515)
    im = imshow(terrain, cmap=cm.hot, origin='lower', extent=extent)

    plot([0.185, 2.815, 0.25, 2.75, 2.5], [0.185, 0.185, 1.972, 1.972,0.672], '+r')
    #axis([0,25,0,25])
    v = axis()
    limits = []
    for val in v:
        limits.append(val)
    limits[0] = -0.022
    limits[1] = 3.0
    limits[2] = -0.022
    limits[3] = 2.122
    axis(limits)
    ###
    plot(d_traj["xRoueGauche"], d_traj["yRoueGauche"])
    hold(True)
    plot(d_traj["xRoueDroite"], d_traj["yRoueDroite"])
    plot(d_traj["xPoseReferenceRobot"][1:], d_traj["yPoseReferenceRobot"][1:],'o')
    grid(True)
    title("trajectoire")
    
    pRef = point(2.815,0.185)
    
    DRP = math.sqrt(pow((d_traj["xRoueGauche"][-1]+d_traj["xRoueDroite"][-1])/2.0 - pRef.x,2)+pow((d_traj["yRoueGauche"][-1]+d_traj["yRoueDroite"][-1])/2.0 - pRef.y,2))
    print("distance rellement parcourue: " + str(DRP)) 
    DPC = math.sqrt(pow(d_traj["xPoseReferenceRobot"][-1] - pRef.x,2)+pow(d_traj["yPoseReferenceRobot"][-1] - pRef.y,2))
    print("distance parcourue de consigne: " + str(DPC)) 
    print("distance sup: " + str(DRP-DPC))
    
    
    periodeHN = d_traj["periode"][0]
    periodeBN = 0.001   #1ms
	
    tempsU = [ periodeHN * x for x in range(len(d_traj["tensionPWM_MoteurGauche"]))]
    
    figure(2)
    plot(tempsU, d_traj["tensionPWM_MoteurGauche"], label="tension moteur gauche")
    hold(True)
    #plot([ periodeBN * x for x in range(len(d_traj["vitesseG1ms"]))], d_traj["vitesseG1ms"], label="tension gauche 1ms")
    plot(tempsU, d_traj["tensionPWM_MoteurDroit"], label="tension moteur droit")
    #plot([ periodeBN * x for x in range(len(d_traj["vitesseD1ms"]))], d_traj["vitesseD1ms"], 'o', markersize=4, label="tension droite 1ms")
    #~ plot(tempsU, d_traj["integPIVG"], label="integG")
    #~ plot(tempsU, d_traj["integPIVD"], label="integD")
    grid(True)
    title("tensions moteurs")
    legend(loc="lower center")
    v = axis()
    limits = []
    for val in v:
        limits.append(val)
    limits[1] = 0.03
    limits[3] = 200
    #axis(limits)
    
    temps = numpy.arange(0.0, (len(d_traj["vitesseMoteurGauche"]))*d_traj["periode"][0], d_traj["periode"][0])
    #tempsVitNorm = numpy.arange(0.0, (len(d_traj["vitesseNormaliseeCourante"]))*d_traj["periode"][0], d_traj["periode"][0])
    #~ print(len(d_traj["ConsigneMoteurDroit"]))
    #print(d_traj["xDerivPos"])
    tempsV = [ periodeHN * x for x in range(len(d_traj["vitesseMoteurGauche"]))]
    figure(3)
    plot(tempsV, d_traj["vitesseMoteurGauche"], label="moteur gauche")
    hold(True)
    #~ plot([ periodeBN * x for x in range(len(d_traj["vitesseG1ms"]))], d_traj["vitesseG1ms"], label="vitesse gauche 1ms")
    plot(tempsV, d_traj["vitesseMoteurDroit"], label="moteur droit")
    #~ plot([ periodeBN * x for x in range(len(d_traj["vitesseD1ms"]))], d_traj["vitesseD1ms"], 'o', markersize=4, label="vitesse droite 1ms")
    #plot(tempsV, d_traj["vitesseNormaliseeCourante"],label="vit norm")
    plot(tempsV, [(d_traj["ConsigneMoteurDroit"][index]-1023)*0.00083 for index in range(len(d_traj["ConsigneMoteurDroit"]))], label="vit cons droit")
    #plot(tempsV, [(d_traj["ConsigneMoteurGauche"][index]-1023)*0.00083 for index in range(len(d_traj["ConsigneMoteurGauche"]))], label="vit cons gauche")
    #plot(temps, d_traj["ConsigneMoteurDroit"], label="vit cons droit")
    #plot(temps, d_traj["ConsigneMoteurGauche"], label="vit cons gauche")
    #plot(tempsV, d_traj["erreurVitesseMoteurDroit"], label="err vit droit")
    #plot(temps, [(d_traj["ConsigneMoteurGauche"][index]-103.0)*0.00089 for index in range(len(d_traj["ConsigneMoteurGauche"]))])
    #plot(temps, [(d_traj["ConsigneMoteurDroit"][index]-1023.0)*0.00089 for index in range(len(d_traj["ConsigneMoteurDroit"]))])
    #~ tempsVit = [ periodeHN * x for x in range(len(d_traj["vitLongitudinale"]))]
    #~ plot(tempsVit, d_traj["vitLongitudinale"], label="vitLong")
    #tempsVitR = [ periodeHN * x for x in range(len(d_traj["vitRotation"]))]
    #plot(tempsVitR, d_traj["vitRotation"], label="vitRot")
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
    #~ plot(tempsPPST, d_traj["distNormaliseeRestante"], label="dist restante")
    #~ # plot([(d_traj["xErreurPoseCentreRobot"][index]/math.fabs(d_traj["xErreurPoseCentreRobot"][index]))*math.sqrt(d_traj["xErreurPoseCentreRobot"][index]*d_traj["xErreurPoseCentreRobot"][index] + d_traj["yErreurPoseCentreRobot"][index]*d_traj["yErreurPoseCentreRobot"][index]) for index in range(len(d_traj["xErreurPoseCentreRobot"]))])
    #~ tempsErrP = [ periodeHN * x for x in range(len(d_traj["xErreur_P"]))]
    #~ # plot(tempsErrP, d_traj["xErreur_P"], label="erreur x")
    #~ # plot(tempsErrP, d_traj["yErreur_P"], label="erreur y")
    #~ 
    #~ tempsEtat = [ periodeHN * x for x in range(len(d_traj["etat"]))]
    #~ plot(tempsEtat, d_traj["etat"], 'or')
    #~ # plot(tempsEtat, d_traj["vitP"], 'o', label="vitP")
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
#~ 
    #~ print("pmax: " + str(d_traj["pmax"]))
    #~ print("errDist: " + str(d_traj["errDist"][-1]))
    #~ 
    #~ print("distance: " + str(d_traj["distance"]))
    #~ 
    #~ # print("len_t: " + str(len(d_traj["tCourbeBS"])))
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

def affichageTestPI(d_traj):
    #duree = d_traj["time"][0]
	
	#nombre de variable loguer
    #print("Nombre de variable loguer: " + str(len(d_traj.keys())))
    
    periode=d_traj["periode"][0]
    temps = [ periode * x for x in range(len(d_traj["vitesseMoteurGauche"]))]
    
    figure(2)
    plot(temps, d_traj["tensionPWM_MoteurGauche"], label="tension moteur gauche")
    hold(True)
    plot(temps, d_traj["tensionPWM_MoteurDroit"], label="tension moteur droit")
    grid(True)
    legend(loc="lower center")
    title("tensions moteurs")
    
    #~ #temps = [temps[index] for index in range(len(temps)-1)]
    print("periode: %s" % periode)
    print("dernier temps: %s" % temps[-1]) 
    print("len_temps: " + str(len(temps)))
    print("len_vitMotG: " + str(len(d_traj["vitesseMoteurGauche"])))
    print("len_consMotG: " + str(len(d_traj["ConsigneMoteurDroit"])))
    print("len_erVitMotG: " + str(len(d_traj["erreurVitesseMoteurDroit"])))
    
    figure(3)
    plot(temps, [(d_traj["ConsigneMoteurGauche"][index]-1023)*0.00083 for index in range(len(d_traj["ConsigneMoteurDroit"]))], label="vit cons gauche")
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

#affichageTraj(trajFunction(T1=4.54, T3=707.0)) #T1=1.57
#affichageTraj(trajFunction(T1=4.43, T2=4.13, T3=712.0))
#affichageTraj(trajFunction(T1=1.23, T2=4.48, T3=800.0))
#print trajFunctionT13([1.32, 859.0])

affichageTraj(trajFunction(Kp = 3.0
							, Ki = 0.0 #2.0
                            , K1 = 20.0
                            , K2 = 50.0
                            , K3 = 20.0
                            , R1 = -6.0
                            , R2 = -6.0
							, T1 = 1.0	# tempsAcc
							, T2 = 4.0	# facteur de vitesse angulaire
							, T3 = 900.0	# Umax
							))
#~ sys.exit(2)
#~ 
#~ affichageTestPI(testPI(Kp = 5.0 #14.0
                        #~ , Ki = 2.0 #27.0
                        #~ , T1=1.0 #1.5
                        #~ , T2=4.48
                        #~ , T3=900.0
                        #~ ))

#~ ecrireCoutPlageKp()
#~ plotCoutPlageKp()
sys.exit(2)
