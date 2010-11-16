#!/usr/bin/env python
# encoding: utf-8

import os

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4 import uic

from mainbar import *

import helpers
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
            data[2] = helpers.translate_packet_data(data[0], data[2])
            self.log_view.append(str(data))
        else:
            self.log_view.append(text)


    def clear(self):
        self.log_view.clear()





class GraphicsRobotItem(QGraphicsItemGroup):

    def __init__(self, parent = None):
        QGraphicsItemGroup.__init__(self, parent)
        rect = QGraphicsRectItem(self.x(), self.y(), 300, 335)
        rect.setBrush(QColor("#777777"))
        self.addToGroup(rect)

        self.rotation_timeline = QTimeLine(2000)
        self.rotation_animation = QGraphicsItemAnimation()
        self.rotation_animation.setItem(self)
        self.rotation_animation.setTimeLine(self.rotation_timeline)


    def robot_rotation(self, angle):
        self.rotation_animation.setRotationAt(1.0, angle)
        self.rotation_timeline.start()

