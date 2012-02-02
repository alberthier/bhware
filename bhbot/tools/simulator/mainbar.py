#!/usr/bin/env python
# encoding: utf-8

import os

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4 import uic

from definitions import *


(MainBar_Ui, MainBar_Widget) =  uic.loadUiType(os.path.join(os.path.dirname(__file__), "mainbar.ui"))




class MainBar(QWidget, MainBar_Ui):

    def __init__(self, parent = None):
        QWidget.__init__(self, parent)
        MainBar_Ui.__init__(self)
        self.setupUi(self)
        self.set_icon(self.reload, "refresh")
        self.set_icon(self.start_pause, "start")
        self.set_icon(self.stop, "stop")
        self.set_color_icon(self.purple_robot, TEAM_COLOR_PURPLE)
        self.set_color_icon(self.red_robot, TEAM_COLOR_RED)

        self.button_group = QButtonGroup(self)
        self.button_group.setExclusive(True)
        self.button_group.addButton(self.basic_dynamics)
        self.button_group.addButton(self.advanced_dynamics)


    def set_icon(self, button, icon_name):
        icons_dir = os.path.join(os.path.dirname(__file__), "icons")
        button.setIcon(QIcon(os.path.join(icons_dir, "{}.svg".format(icon_name))))


    def set_color_icon(self, button, color):
        pixmap = QPixmap(QSize(16, 16))
        pixmap.fill(QColor(color))
        button.setIcon(QIcon(pixmap))
