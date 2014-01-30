# encoding: utf-8

import os
import math

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4 import uic

from mainbar import *

import leds
import logger
from definitions import *
import sys

(OutputView_Ui, OutputView_Widget) = uic.loadUiType(os.path.join(os.path.dirname(__file__), "outputview.ui"))




class OutputView(QWidget, OutputView_Ui):

    COLORS = {logger.TERM_BLACK   : "#000000",
              logger.TERM_RED     : "#a81300",
              logger.TERM_GREEN   : "#005400",
              logger.TERM_YELLOW  : "#c36806",
              logger.TERM_BLUE    : "#1d295a",
              logger.TERM_MAGENTA : "#66004b",
              logger.TERM_CYAN    : "#006f84",
              logger.TERM_WHITE   : "#ffffff"}

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
        self.log_view.setTextColor(self.default_color)
        self.log_view.setFontWeight(QFont.Normal)
        has_ec = False
        while text.startswith('\033'):
            has_ec = True
            i = text.find('m')
            ec = text[: i + 1]
            if ec in OutputView.COLORS:
                self.log_view.setTextColor(QColor(OutputView.COLORS[ec]))
            if ec == logger.TERM_BOLD:
                self.log_view.setFontWeight(QFont.Bold)
            text = text[i + 1 :]
        if has_ec:
            text = text[:text.rfind('\033')]

        self.log_view.append(text)



    def setup(self, team, is_main):
        self.log_view.clear()
        palette = QPalette()
        if team is not None:
            if team == TEAM_RED:
                self.color = QColor(TEAM_COLOR_RED)
            elif team == TEAM_YELLOW:
                self.color = QColor(TEAM_COLOR_YELLOW)
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
