#!/usr/bin/env python
# encoding: utf-8


import os
import imp
import math

from PyQt4.QtCore import *

import trajectory
import tools
from definitions import *




class BasicDynamics(QObject):

    simulation_finished = pyqtSignal(list)

    def __init__(self):
        QObject.__init__(self)
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0


    def resettle(self, packet):
        if packet.axis == AXIS_X:
            self.x = packet.position
        elif packet.axis == AXIS_Y:
            self.y = packet.position
        self.angle = packet.angle


    def goto(self, packet):
        traj = []
        linear_speed = 0.60 # m/s
        angular_speed = math.pi * 2.0 # rad/s

        for pose in packet.points:
            d = tools.distance(self.x, self.y, pose.x, pose.y)
            if not tools.quasi_null(d):
                time = d / linear_speed
            else:
                a = abs(self.angle - pose.angle)
                time = a / angular_speed
            traj.append((time, pose))
            self.x = pose.x
            self.y = pose.y
            self.angle = pose.angle

        self.simulation_finished.emit(traj)


    def terminate(self):
        pass




class PositionControlSimulatorDynamics(QObject):

    simulation_finished = pyqtSignal(list)

    def __init__(self, parent = None):
        QObject.__init__(self, parent)

        self.left_x = 0.0
        self.left_y = 0.0
        self.right_x = 0.0
        self.poses = []

        self.simulator_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
        self.simulator_dir = os.path.join(self.simulator_dir, "asser")
        file_path = os.path.join(self.simulator_dir, "simulator_trajAsser")
        self.process = QProcess(self)
        self.process.setReadChannel(QProcess.StandardOutput)

        self.process.readyRead.connect(self.read_stdout)

        self.process.start(file_path)
        self.process.waitForStarted()

        module = imp.load_source("params_module", os.path.join(self.simulator_dir, "config.py"))
        params = module.d_cfgTraj
        self.process.write("PARAMETERS_PI 0 0 2 KP {} KI {}\n\n".format(params["Kp"], params["Ki"]))
        self.process.write("PARAMETERS_GAIN_K 0 0 3 GAIN_K1 {} GAIN_K2 {} GAIN_K3 {}\n\n".format(params["K1"], params["K2"], params["K3"]))
        self.process.write("PARAMETERS_GAIN_ROT 0 0 2 GAIN_R1 {} GAIN_R2 {}\n\n".format(params["R1"], params["R2"]))
        self.process.write("PARAMETERS_TIME 0 0 3 ACC {} VITANGMAX {} UMAX {}\n\n".format(params["TempsAcc"], params["Facteur_vitesse_angulaire"], params["Umax"]))
        self.process.write("PARAMETERS_MOTOR 0 0 9 MASSE {} RAYON_ROUE {} FROTTEMENT_FLUIDE {} FORCE_RESISTANTE {} RESISTANCE_INDUIT {} INDUCTANCE_INDUIT {} CONSTANTE_COUPLE {} CONSTANTE_VITESSE {} RAPPORT_REDUCTION {}\n\n".format(params["Masse"], params["Rayon_roue"], params["Frottement_fluide"], params["Force_resistante"], params["Resistance_induit"], params["Inductance_induit"], params["Constance_couple"], params["Constante_vitesse"], params["Rapport_reduction"]))
        self.process.waitForBytesWritten()


    def read_stdout(self):
        while self.process.canReadLine():
            output = str(self.process.readLine())
            if output.startswith("log_xRoueGauche:"):
                self.left_x = float(output[output.find(":") + 1 : ].strip())
            elif output.startswith("log_yRoueGauche:"):
                self.left_y = float(output[output.find(":") + 1 : ].strip())
            elif output.startswith("log_xRoueDroite:"):
                self.right_x = float(output[output.find(":") + 1 : ].strip())
            elif output.startswith("log_yRoueDroite:"):
                right_y = float(output[output.find(":") + 1 : ].strip())
                x = abs(self.left_x + self.right_x) / 2.0
                y = abs(self.left_y + right_y) / 2.0
                angle = math.atan2(self.left_y - right_y, self.left_x - self.right_x)
                self.poses.append(trajectory.Pose(x, y, angle))
            elif output.startswith("Motion"):
                poses = []
                for i in xrange(0, len(self.poses), 10):
                    poses.append(self.poses[i])
                self.simulation_finished.emit(poses)
                self.poses = []


    def initialize(self, x, y, angle):
        self.process.write("INIT_POSE_ROBOT 0 0 1 {} {} {} 1.0\n\n".format(x, y, angle))


    def goto(self, packet):
        self.poses = []
        cmd = "MSG_MAIN_GOTO {} {} {}".format(packet.movement, packet.direction, len(packet.points))
        for pose in packet.points:
            cmd += " {} {}".format(pose.x, pose.y)
            if pose.angle != None:
                cmd += " {} 1.0".format(pose.angle)
            else:
                cmd += " 0.0 0.0"
        cmd += "\n\n"
        self.process.write(cmd)


    def terminate(self):
        self.process.write("QUIT\n\n")
        self.process.waitForFinished()
