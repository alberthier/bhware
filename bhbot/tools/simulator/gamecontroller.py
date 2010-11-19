#!/usr/bin/env python
# encoding: utf-8

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtNetwork import *

import config
from definitions import *

from robotcontroller import *



class GameController(object):

    def __init__(self, main_window):
        self.server = QTcpServer()
        self.server.newConnection.connect(self.brewery_connected)
        self.server.listen(QHostAddress.Any, config.remote_port)
        self.red_robot = RobotController(TEAM_RED, self, main_window.red_robot_dock.widget(), main_window.field_scene)
        self.blue_robot = RobotController(TEAM_BLUE, self, main_window.blue_robot_dock.widget(), main_window.field_scene)
        self.main_bar = main_window.main_bar_dock.widget()
        self.main_bar.start.clicked.connect(self.setup)
        self.main_bar.stop.clicked.connect(self.stop)
        QCoreApplication.instance().lastWindowClosed.connect(self.stop)
        self.is_started = False
        self.time = 0



    def setup(self):
        if not self.red_robot.is_started():
            self.red_robot.setup()
        elif not self.blue_robot.is_started():
            self.blue_robot.setup()


    def stop(self):
        self.red_robot.stop()
        self.blue_robot.stop()


    def brewery_connected(self):
        if not self.red_robot.is_connected():
            self.red_robot.connected(self.server.nextPendingConnection())
            self.setup()
        elif not self.blue_robot.is_connected():
            self.blue_robot.connected(self.server.nextPendingConnection())
            self.setup()
        else:
            # Only two robots can play
            self.server.nextPendingConnection().close()
            return
