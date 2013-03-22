# encoding: utf-8

import os
import math

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4 import uic

from mainbar import *

import leds
from definitions import *
import sys

(OutputView_Ui, OutputView_Widget) = uic.loadUiType(os.path.join(os.path.dirname(__file__), "outputview.ui"))




class OutputView(QWidget, OutputView_Ui):

    def __init__(self, parent):
        QWidget.__init__(self, parent)
        OutputView_Ui.__init__(self)
        self.setupUi(self)

        font = None
        if sys.platform == 'darwin' :
            font = QFont("Monaco", 8)
        else :
            font = QFont("", 8)
        font.setStyleHint(QFont.TypeWriter)
        self.log_view.setCurrentFont(font)

        self.default_color = self.log_view.textColor()
        self.setup(None, None)

        self.led_off_color = QPalette().color(QPalette.Window)
        self.led_orange_color = QColor("#f57900")
        self.led_green_color = QColor("#73d216")


    def add_log(self, text):
        if text.startswith("["):
            self.log_view.setTextColor(self.default_color)
        else:
            self.log_view.setTextColor(self.color)
        self.log_view.append('{}'.format(text))
        self.log_view.setTextColor(self.default_color)


    def setup(self, team, is_main):
        self.log_view.clear()
        palette = QPalette()
        if team is not None:
            if team == TEAM_RED:
                self.color = QColor(TEAM_COLOR_RED)
            elif team == TEAM_BLUE:
                self.color = QColor(TEAM_COLOR_BLUE)
            color = self.color
            if not is_main:
                color = color.lighter()
            palette.setColor(QPalette.Window, QColor(color))
        self.setPalette(palette)


    def handle_led(self, led_data):
        if (led_data & leds.SimulatorLed.COLOR_ORANGE_FLAG) and (led_data & leds.SimulatorLed.COLOR_ORANGE):
            color = self.led_orange_color
        elif (led_data & leds.SimulatorLed.COLOR_GREEN_FLAG) and (led_data & leds.SimulatorLed.COLOR_GREEN):
            color = self.led_green_color
        else:
            color = self.led_off_color

        palette = self.led.palette()
        palette.setColor(QPalette.Window, color)
        self.led.setPalette(palette)
