#!/usr/bin/env python
# encoding: utf-8

import os

from PyQt4.QtGui import *
from PyQt4 import uic
from PyQt4.Qt import Qt

from mainbar import *
from robotview import *
from gamecontroller import *

from definitions import *

(MainWindow_Ui, MainWindow_Widget) = uic.loadUiType(os.path.join(os.path.dirname(__file__), "mainwindow.ui"))




class MainWindow(QMainWindow, MainWindow_Ui):

    def __init__(self, parent = None):
        QMainWindow.__init__(self, parent)
        MainWindow_Ui.__init__(self)
        self.setupUi(self)
        self.setWindowIcon(QIcon.fromTheme("applications-development"))

        self.main_bar_dock.setWidget(MainBar(self))
        self.red_robot_dock.setWidget(RobotView(self, TEAM_RED))
        self.blue_robot_dock.setWidget(RobotView(self, TEAM_BLUE))

        self.game_controller = GameController(self)
