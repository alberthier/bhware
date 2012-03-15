#!/usr/bin/env python
# encoding: utf-8


import os
import imp
import math
import time

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


    def setup(self):
        pass


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
        segmentNb = 0

        for pose in packet.points:
            d = tools.distance(self.x, self.y, pose.x, pose.y)
            if not tools.quasi_null(d):
                time = d / linear_speed
            else:
                a = (self.angle  - pose.angle) % (2.0 * math.pi)
                if a > math.pi:
                    a -= math.pi
                time = a / angular_speed
            traj.append((segmentNb, time, pose))
            self.x = pose.x
            self.y = pose.y
            self.angle = pose.angle
            segmentNb += 1

        self.simulation_finished.emit(traj)


    def terminate(self):
        pass




class PositionControlSimulatorDynamics(QObject):

    TIMER_RESOLUTION = 0.010

    simulation_finished = pyqtSignal(list)

    def __init__(self, parent = None):
        QObject.__init__(self, parent)

        self.left_x = 0.0
        self.left_y = 0.0
        self.right_x = 0.0
        self.right_y = 0.0
        self.previous_pose = trajectory.Pose()
        self.current_pose = trajectory.Pose()
        self.current_segment_nb = None
        self.elapsed_time = 0.0
        self.poses = []

        self.simulator_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
        self.simulator_dir = os.path.join(self.simulator_dir, "asser")
        self.process = None

        self.build()


    def setup(self):
        self.build()
        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.MergedChannels)

        self.process.readyRead.connect(self.read_stdout)

        file_path = os.path.join(self.simulator_dir, "simulator_trajAsser")
        self.process.start(file_path)
        self.process.waitForStarted()
        # Wait until the simulator is properly initialized.
        self.process.waitForReadyRead()


    def build(self):
        sources = ["asserv_trajectoire.c", "mainSimuAsserBin.c", "position.c", "simuAsser.c", "simu_task_asser.c", "pic18.c"]
        binary = "simulator_trajAsser"
        requires_link = False
        binary_path = os.path.join(self.simulator_dir, binary)
        if os.path.exists(binary_path):
            binary_mtime = os.stat(binary_path).st_mtime
        else:
            binary_mtime = 0
        for source in sources:
            mtime = os.stat(os.path.join(self.simulator_dir, source)).st_mtime
            if mtime > binary_mtime:
                args = ["-o", binary] + sources + [ "-lm" ]
                comp = QProcess()
                comp.setWorkingDirectory(self.simulator_dir)
                comp.start("gcc", args)
                comp.waitForFinished()
                if comp.exitCode() == 0:
                    print("Compilation succeeded")
                else:
                    print("Compilation failed")
                break


    def resettle(self, packet):
        if packet.axis == AXIS_X:
            self.previous_pose.x = packet.position
        elif packet.axis == AXIS_Y:
            self.previous_pose.y = packet.position
            self.previous_pose.angle = packet.angle
            self.invoke_move("INIT_POSE_ROBOT", 0, 0, [self.previous_pose])


    def goto(self, packet):
        self.poses = []
        self.elapsed_time = 0.0
        self.invoke_move("MSG_MAIN_GOTO", packet.movement, packet.direction, packet.points)


    def terminate(self):
        self.process.write("QUIT\n")
        self.process.waitForFinished()


    def invoke_setup(self, function_name, *args):
        cmd = function_name
        cmd += " " + str(len(args) / 2)
        for arg in args:
            cmd += " " + str(arg)
        cmd += "\n"
        self.process.write(cmd)
        self.process.waitForBytesWritten()


    def invoke_move(self, function_name, movement, direction, points):
        cmd = function_name + " " + str(movement) + " " + str(direction)
        cmd += " " + str(len(points))
        for pose in points:
            cmd += " " + str(pose.x) + " " + str(pose.y)
            if pose.angle != None:
                cmd += " " + str(pose.angle) + " 1.0"
            else:
                cmd += " 0.0 0.0"
        cmd += "\n"
        self.process.write(cmd)
        self.process.waitForBytesWritten()


    def read_value(self, line):
        return float(line[line.find(":") + 1 : ].strip())


    def read_stdout(self):
        while self.process.canReadLine():
            output = str(self.process.readLine())
            if output.startswith("log_xRoueGauche:"):
                self.left_x = self.read_value(output)
            elif output.startswith("log_yRoueGauche:"):
                self.left_y = self.read_value(output)
            elif output.startswith("log_xRoueDroite:"):
                self.right_x = self.read_value(output)
            elif output.startswith("log_yRoueDroite:"):
                self.right_y = self.read_value(output)
            elif output.startswith("log_segmentCourant:"):
                self.current_segment_nb = int(self.read_value(output))
                self.current_pose.x = abs(self.left_x + self.right_x) / 2.0
                self.current_pose.y = abs(self.left_y + self.right_y) / 2.0
                self.current_pose.angle = math.atan2(self.right_x - self.left_x, self.left_y - self.right_y)
                #self.elapsed_time += PositionControlSimulatorDynamics.TIMER_RESOLUTION
                if tools.distance(self.previous_pose.x, self.previous_pose.y, self.current_pose.x, self.current_pose.y) > 0.100 or abs(self.previous_pose.angle - self.current_pose.angle) > 0.20:
                    self.previous_pose.x = self.current_pose.x
                    self.previous_pose.y = self.current_pose.y
                    self.previous_pose.angle = self.current_pose.angle
                    self.poses.append((self.current_segment_nb, self.elapsed_time, trajectory.Pose(self.current_pose.x, self.current_pose.y, self.current_pose.angle)))
                    self.current_segment_nb = None
                    self.elapsed_time += PositionControlSimulatorDynamics.TIMER_RESOLUTION * 10.0
            elif output.startswith("Motion"):
                if self.current_segment_nb != None:
                    self.poses.append((self.current_segment_nb, self.elapsed_time, trajectory.Pose(self.current_pose.x, self.current_pose.y, self.current_pose.angle)))
                    self.current_segment_nb = None
                self.simulation_finished.emit(self.poses)
                self.poses = []
