#!/usr/bin/env python
# encoding: utf-8

import os

from PyQt4.QtGui import *
from PyQt4 import uic

from mainbar import *

import logviewer
from definitions import *

(RobotView_Ui, RobotView_Widget) = uic.loadUiType(os.path.join(os.path.dirname(__file__), "robotview.ui"))




class RobotView(QWidget, RobotView_Ui):

    def __init__(self, parent, team):
        QWidget.__init__(self, parent)
        RobotView_Ui.__init__(self)
        self.setupUi(self)

        if team == TEAM_RED:
            color = "#c90000"
        elif team == TEAM_BLUE:
            color = "#0000c9"
        palette = self.palette()
        palette.setColor(QPalette.Window, QColor(color))
        palette.setColor(QPalette.Window, QColor(color))
        palette.setColor(QPalette.Window, QColor(color))
        self.setPalette(palette)


    def add_log(self, text):
        if text.startswith("["):
            # this line is a packet
            data = eval(text)
            data[2] = logviewer.format_log_line(data[0], data[2])
            self.log_view.append(str(data))
        else:
            self.log_view.append(text)


    def clear(self):
        self.log_view.clear()

