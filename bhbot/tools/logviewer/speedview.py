# encoding: utf-8

import math

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtSvg import *

import logger
import packets
from definitions import *

import logtools




class Graph(QGraphicsItemGroup):

    def __init__(self, controller):
        QGraphicsItemGroup.__init__(self)
        self.addToGroup(QGraphicsLineItem(self.get_x(0.0),
                                          self.get_y(-0.1),
                                          self.get_x(0.0),
                                          self.get_y(1.1)))
        lines = 4
        for i in range(lines + 1):
            c = i / float(lines)
            self.addToGroup(QGraphicsLineItem(-10, self.get_y(c), self.get_x(len(controller.speeds)), self.get_y(c)))
        lines = 10
        for i in range(lines):
            c = float((i + 1) * len(controller.speeds)) / float(lines)
            self.addToGroup(QGraphicsLineItem(self.get_x(c), self.get_y(0.0), self.get_x(c), self.get_y(-0.1)))
        path = QPainterPath()
        path.moveTo(self.get_x(0.0), self.get_y(0.0))
        x = 0
        for s in controller.speeds:
            path.lineTo(self.get_x(x), self.get_y(s))
            x += 1
        path.lineTo(self.get_x(x - 1), self.get_y(0.0))
        item = QGraphicsPathItem(path)
        item.setPen(QPen(QColor("#1B609E"), 2))
        item.setBrush(QColor("#2279C7"))
        self.addToGroup(item)


    def get_x(self, value):
        return value * 5.0

    def get_y(self, value):
        return 100.0 - (value * 100.0)




class SpeedViewController(QObject):

    def __init__(self, ui, parent = None):
        QObject.__init__(self)
        self.ui = ui

        self.points = []
        self.speeds = []

        self.max_speed = 0.0
        self.average_speed = 0.0

        self.scene = QGraphicsScene()

        self.ui.speed_view.setScene(self.scene)


    def process_log_line(self, log_line, lineno, last_lineno):
        packet_type = log_line[logger.LOG_LINE_PACKET]
        sender = log_line[logger.LOG_LINE_SENDER]
        if packet_type is packets.KeepAlive and sender == "PIC":
            pose = logtools.get_value(log_line[logger.LOG_LINE_CONTENT], "current_pose")
            x = logtools.get_value(pose, "x")
            y = logtools.get_value(pose, "y")
            if len(self.points) > 0:
                oldx, oldy = self.points[-1]
                speed = math.sqrt((x - oldx) ** 2 + (y - oldy) ** 2) / (KEEP_ALIVE_DELAY_MS / 1000.0)
                self.speeds.append(speed)
                if speed > self.max_speed:
                    self.max_speed = speed
                self.average_speed += speed
            self.points.append((x, y))
        if lineno == last_lineno:
            self.average_speed / float(len(self.speeds))


    def log_loaded(self):
        self.graph = Graph(self)
        self.scene.addItem(self.graph)
        #r = self.graph.boundingRect().normalized()
        #self.scene.setSceneRect(r)
        #self.ui.speed_view.fitInView(r, Qt.KeepAspectRatio)
