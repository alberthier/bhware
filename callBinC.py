#!/usr/bin/python

import subprocess
import sys
import time
from matplotlib.pyplot import *
import numpy


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
            if l_linedata[0].count('log_', 0, 4) == 1:
                l_linedata[0] = l_linedata[0].replace("log_", "")
                l_linedata[0] = l_linedata[0].replace(":", "")
                if d_traj.keys().count(l_linedata[0]) == 0:
                    d_traj[l_linedata[0]] = []
                d_traj[l_linedata[0]].append(float(l_linedata[1]))
            
    return d_traj
    
def trajFunction(K1 = 20.0, K2 = 75.0, K3 = 50.0, R1 = -10.2, R2 = -10.2, T1 = 3.0, T2 = 4.0, T3 = 900):
    #generation d'un message d'init de pose
    poseInit = commandMsg("INIT_POSE_ROBOT 0 0")
    poseInit.addPose("0.0 0.0 0.0")
    initPoseMessage = poseInit.cmdMsgGeneration()
    
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
    print "UMAX " + str(T3)
    msg_parametersT = parametersT.cmdMsgGeneration()
    
    #generation d'un message de commande de deplacement
    dep1 = commandMsg("MSG_MAIN_GOTO 1 1")   # 'DEPLACEMENT' en 'MARCHE_AVANT'
    dep1.addPose("1.0 0.1 0.0")
    dep1.addPose("1.5 0.3 0.3")
    #dep1.addPose("1.8 0.2 0.7")
    #dep.addPose("1.7 1.7 0.4")
    depMessage1 = dep1.cmdMsgGeneration()
    
    #dep2 = commandMsg("MSG_MAIN_GOTO 1 1")   # 'DEPLACEMENT' en 'MARCHE_AVANT'
    #dep2.addPose("1.8 0.35 0.1")
    #dep.addPose("1.5 1.2 0.4")
    #dep.addPose("1.6 1.3 0.4")
    #dep.addPose("1.7 1.7 0.4")
    #depMessage2 = dep2.cmdMsgGeneration()
    
    #lancement du simulateur de deplacement
    simulator_process = subprocess.Popen('./simulator_trajAsser', shell=True, stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.PIPE)
    
    #initilisation des parametres de l'asser de trajectoire
    simulator_process.stdin.write(msg_parametersK)
    simulator_process.stdin.write(msg_parametersR)
    simulator_process.stdin.write(msg_parametersT)
    
    #transmission de l'init de pose par l'entree standard
    simulator_process.stdin.write(initPoseMessage)
    
    #transmission de commandes de deplacement par l'entree standard
    simulator_process.stdin.write(depMessage1)
    #simulator_process.stdin.write(depMessage2)
    
    #transmission de la commande de d'arret du simulateur
    (stdoutdata, stderrdata) = simulator_process.communicate("QUIT\n")
    
    #print 'len stdout:', len(stdoutdata)
    
    #traitement des donnees de stdout
    lines = lineForm(stdoutdata)
    d_traj = stdoutParser(lines)
    print("UMAX: " + str(d_traj["UMAX"][0]))
    
    return lines[-3]
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

print trajFunction(T1=0.04)
