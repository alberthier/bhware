#!/usr/bin/env python
# encoding: utf-8

import os

from PyQt4.QtCore import *




class RobotController(object):

    def __init__(self, team, view):
        self.team = team
        self.view = view
        self.process = None
        self.socket = None


    def is_started(self):
        return self.process != None


    def is_connected(self):
        return self.socket != None


    def setup(self):
        if self.process == None:
            self.view.clear()
            brewery = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "brewery", "brewery.py")
            self.process = QProcess()
            self.process.setReadChannelMode(QProcess.MergedChannels)
            self.process.readyRead.connect(self.read_output)
            self.process.start(brewery, [])


    def stop(self):
        if self.process != None:
            self.process.kill()
            self.process.waitForFinished()
            self.process = None
            self.socket = None


    def connected(self, socket):
        self.socket = socket


    def read_output(self):
        while self.process.canReadLine():
            log = str(self.process.readLine()).rstrip()
            self.view.add_log(log)
