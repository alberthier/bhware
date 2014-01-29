# encoding: utf-8

import os

from PyQt4.QtGui import *
from PyQt4 import uic
from PyQt4.Qt import Qt

from mainbar import *
from outputview import *
from gamecontroller import *
from simulatorfieldview import *

from definitions import *

(MainWindow_Ui, MainWindow_Widget) = uic.loadUiType(os.path.join(os.path.dirname(__file__), "mainwindow.ui"))




class MainWindow(QMainWindow, MainWindow_Ui):

    def __init__(self, args, parent = None):
        QMainWindow.__init__(self, parent)
        MainWindow_Ui.__init__(self)
        self.setupUi(self)
        self.setWindowIcon(QIcon(os.path.join(os.path.dirname(__file__), 'icons/main.png')))

        self.main_bar = MainBar(self)
        self.main_bar_container_layout.addWidget(self.main_bar)
        self.robot_a_output_view = OutputView(self)
        self.robot_a_output_view_container_layout.addWidget(self.robot_a_output_view)
        self.robot_b_output_view = OutputView(self)
        self.robot_b_output_view_container_layout.addWidget(self.robot_b_output_view)

        self.field_view_controller = SimulatorFieldViewController(self)
        self.game_controller = GameController(self, args)

