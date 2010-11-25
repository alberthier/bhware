#!/usr/bin/env python
# encoding: utf-8

import os
import math

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtSvg import *
from PyQt4 import uic

from mainbar import *

import helpers
import leds
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
        self.setPalette(palette)

        self.points_widget.setPalette(self.style().standardPalette())


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


    def handle_led(self, led_data):
        color = QApplication.instance().palette().color(QPalette.Window)
        if (led_data & leds.SimulatorLed.COLOR_ORANGE_FLAG) and (led_data & leds.SimulatorLed.COLOR_ORANGE):
            color = QColor("#f57900")
        elif (led_data & leds.SimulatorLed.COLOR_GREEN_FLAG) and (led_data & leds.SimulatorLed.COLOR_GREEN):
            color = QColor("#73d216")
        palette = self.led.palette()
        palette.setColor(QPalette.Window, color)
        self.led.setPalette(palette)




class GraphicsRobotItem(QGraphicsItemGroup):

    def __init__(self, parent = None):
        QGraphicsItemGroup.__init__(self, parent)
        self.robot = QGraphicsSvgItem(os.path.join(os.path.dirname(__file__), "robot.svg"))
        self.robot.translate(-260.5, -170.0)
        self.addToGroup(self.robot)

        self.timeline = QTimeLine()
        self.animation = QGraphicsItemAnimation()
        self.animation.setItem(self)
        self.animation.setTimeLine(self.timeline)


    def robot_rotation(self, angle):
        # 360 deg/s
        self.timeline.setDuration(abs(int(1000.0 / 360.0 * angle)))
        self.animation.setRotationAt(1.0, angle)
        self.timeline.start()


    def robot_move(self, dx, dy):
        # 1 m/s
        dist = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
        self.timeline.setDuration(dist)
        self.animation.setTranslationAt(1.0, dx, dy)
        self.timeline.start()
