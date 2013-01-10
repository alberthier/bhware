# encoding: utf-8

import os

from PyQt4.QtGui import *
from PyQt4 import uic
from PyQt4.Qt import Qt

from mainbar import *
from robotview import *
from gamecontroller import *
from simulatorfieldview import *

from definitions import *

(MainWindow_Ui, MainWindow_Widget) = uic.loadUiType(os.path.join(os.path.dirname(__file__), "mainwindow.ui"))




class MainWindow(QMainWindow, MainWindow_Ui):

    def __init__(self, piece_config, parent = None):
        QMainWindow.__init__(self, parent)
        MainWindow_Ui.__init__(self)
        self.setupUi(self)
        self.setWindowIcon(QIcon(os.path.join(os.path.dirname(__file__),'icons/main.png')))

        self.main_bar = MainBar(self)
        self.set_widget(self.main_bar_container, self.main_bar)
        self.blue_robot_view = RobotView(self, TEAM_BLUE)
        self.set_widget(self.blue_robot_view_container, self.blue_robot_view)
        self.red_robot_view = RobotView(self, TEAM_RED)
        self.set_widget(self.red_robot_view_container, self.red_robot_view)

        self.field_controller = SimulatorFieldViewController(self)
        self.game_controller = GameController(self)


    def set_widget(self, widget, child_widget):
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(child_widget)
