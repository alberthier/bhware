#!/usr/bin/env python
# encoding: utf-8

import os
import math

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4 import uic

from mainbar import *

import leds
from definitions import *

(RobotView_Ui, RobotView_Widget) = uic.loadUiType(os.path.join(os.path.dirname(__file__), "robotview.ui"))




class RobotView(QWidget, RobotView_Ui):

    def __init__(self, parent, team):
        QWidget.__init__(self, parent)
        RobotView_Ui.__init__(self)
        self.setupUi(self)

        if team == TEAM_RED:
            self.color = TEAM_COLOR_RED
        elif team == TEAM_PURPLE:
            self.color = TEAM_COLOR_PURPLE
        palette = self.palette()
        palette.setColor(QPalette.Window, QColor(self.color))
        self.setPalette(palette)


    def add_log(self, text):
        if text.startswith("["):
            self.log_view.append('<font size="6">{}</font>'.format(text))
        else:
            self.log_view.append('<font size="6" color="{}">{}</font>'.format(self.color, text))


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
