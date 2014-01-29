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
        self.set_color_icon(self.main_yellow_robot, TEAM_COLOR_YELLOW)
        self.set_color_icon(self.secondary_yellow_robot, TEAM_COLOR_YELLOW)
        self.set_color_icon(self.main_red_robot, TEAM_COLOR_RED)
        self.set_color_icon(self.secondary_red_robot, TEAM_COLOR_RED)

        self.main_yellow_robot.setChecked(True)
        self.main_red_robot.setChecked(True)
        self.oldest_pressed = self.main_yellow_robot

        self.main_yellow_robot.toggled.connect(self.main_yellow_toggled)
        self.secondary_yellow_robot.toggled.connect(self.secondary_yellow_toggled)
        self.main_red_robot.toggled.connect(self.main_red_toggled)
        self.secondary_red_robot.toggled.connect(self.secondary_red_toggled)


    def set_icon(self, button, icon_name):
        icons_dir = os.path.join(os.path.dirname(__file__), "icons")
        button.setIcon(QIcon(os.path.join(icons_dir, "{}.svg".format(icon_name))))


    def set_color_icon(self, button, color):
        pixmap = QPixmap(QSize(16, 16))
        pixmap.fill(QColor(color))
        button.setIcon(QIcon(pixmap))


    def main_yellow_toggled(self):
        self.button_toggled(self.main_yellow_robot)


    def secondary_yellow_toggled(self):
        self.button_toggled(self.secondary_yellow_robot)


    def main_red_toggled(self):
        self.button_toggled(self.main_red_robot)


    def secondary_red_toggled(self):
        self.button_toggled(self.secondary_red_robot)


    def button_toggled(self, toggled):
        buttons = [ self.main_yellow_robot, self.secondary_yellow_robot, self.main_red_robot, self.secondary_red_robot ]
        cpt = 0
        for button in buttons:
            if button.isChecked():
                cpt += 1
        if cpt > 2:
            self.oldest_pressed.setChecked(False)
        for button in buttons:
            if button != toggled and button.isChecked():
                self.oldest_pressed = button
                break
        if self.oldest_pressed is None:
            self.oldest_pressed = toggled


    def get_expected_robots(self):
        expected = []
        buttons = [ self.main_yellow_robot, self.secondary_yellow_robot, self.main_red_robot, self.secondary_red_robot ]
        teams = [ TEAM_YELLOW, TEAM_YELLOW, TEAM_RED, TEAM_RED ]
        robot_type = [ True, False, True, False ]

        for button, team, is_main in zip(buttons, teams, robot_type):
            if button.isChecked():
                expected.append((team, is_main))

        return expected
