#!/usr/bin/env python
# encoding: utf-8

import os

from PyQt4.QtGui import *
from PyQt4 import uic
from PyQt4.Qt import Qt

from mainbar import *
from robotview import *
from gamecontroller import *
from field import *

from definitions import *

(MainWindow_Ui, MainWindow_Widget) = uic.loadUiType(os.path.join(os.path.dirname(__file__), "mainwindow.ui"))




class MainWindow(QMainWindow, MainWindow_Ui):

    def __init__(self, piece_config, parent = None):
        QMainWindow.__init__(self, parent)
        MainWindow_Ui.__init__(self)
        self.setupUi(self)
        # self.setWindowIcon(QIcon.fromTheme("applications-development"))
        self.setWindowIcon(QIcon(os.path.join(os.path.dirname(__file__),'icons/main.png')))

        main_bar = MainBar(self)
        self.main_bar_dock.setWidget(main_bar)
        self.red_robot_dock.setWidget(RobotView(self, TEAM_RED))
        self.blue_robot_dock.setWidget(RobotView(self, TEAM_BLUE))

        self.field_scene = FieldScene(piece_config, main_bar)
        self.setCentralWidget(FieldView(self.field_scene, self))

        self.game_controller = GameController(self)
