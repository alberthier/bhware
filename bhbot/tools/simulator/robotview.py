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
import trajectory
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




class GraphicsRobotObject(QObject):

    def __init__(self, team, parent = None):
        QObject.__init__(self, parent)

        self.team = team
        self.animation = QPropertyAnimation()
        self.animation.setTargetObject(self)

        self.item = QGraphicsItemGroup()
        self.robot_item = QGraphicsSvgItem(os.path.join(os.path.dirname(__file__), "robot.svg"))
        self.robot_item.setTransformOriginPoint(260.5, 170.0)
        transform = self.robot_item.transform()
        transform.translate(-260.5, -170.0)
        self.robot_item.setTransform(transform)
        self.item.addToGroup(self.robot_item)
        team_indicator = QGraphicsEllipseItem(-25.0, 15.0, 50.0, 50.0)
        if self.team == TEAM_RED:
            color = QColor("#c90000")
        elif self.team == TEAM_BLUE:
            color = QColor("#0000c9")
        team_indicator.setBrush(color)
        team_indicator.setPen(QPen(0))
        self.item.addToGroup(team_indicator)


    def get_position(self):
        return self.item.pos()


    def set_position(self, p):
        self.item.setPos(p)

    # declare 'position' to Qt's property system
    position = pyqtProperty('QPointF', get_position, set_position)

    def get_rotation(self):
        return self.item.rotation()


    def set_rotation(self, a):
        self.item.setRotation(a)

    #declare 'angle' to Qt's property system
    angle = pyqtProperty('qreal', get_rotation, set_rotation)


    def convert_angle(self, angle):
        return math.atan2(math.cos(angle), math.sin(angle))


    def get_pose(self):
        y = self.item.pos().x() / 1000.0
        x = self.item.pos().y() / 1000.0
        angle = self.item.rotation() / 180.0 * math.pi
        # Map from Qt reference to field reference
        angle = self.convert_angle(angle)
        return trajectory.Pose(x, y, angle)


    def robot_rotation(self, angle):
        # Map from robot field reference to Qt reference
        ref_angle = self.convert_angle(angle)
        angle_deg = ((ref_angle) / math.pi * 180.0)

        current = self.item.rotation() % 360.0

        if abs(current - angle_deg) > 180.0:
            if current > angle_deg:
                angle_deg += 360.0
            else:
                angle_deg -= 360.0

        self.animation.setPropertyName("angle")
        # 360 deg/s
        duration = abs(int((angle_deg - current) / 360.0 * 1000.0))
        self.animation.setDuration(duration)
        self.animation.setStartValue(current)
        self.animation.setEndValue(angle_deg)
        self.animation.start()


    def robot_move(self, x, y):
        # Map from robot field reference to Qt reference
        ref_x = y * 1000.0
        ref_y = x * 1000.0
        d_field_x = ref_x - self.item.pos().x()
        d_field_y = ref_y - self.item.pos().y()

        self.animation.setPropertyName("position")
        # 1 m/s
        duration = math.sqrt(math.pow(d_field_x, 2) + math.pow(d_field_y, 2))
        self.animation.setDuration(duration)
        self.animation.setStartValue(self.item.pos())
        dest = QPointF(ref_x, ref_y)
        self.animation.setEndValue(dest)
        self.animation.start()
