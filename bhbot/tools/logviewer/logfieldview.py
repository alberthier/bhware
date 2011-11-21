#!/usr/bin/env python
# encoding: utf-8


from PyQt4.QtCore import *
from PyQt4.QtGui import *

import logger
import packets
from definitions import *

import fieldview




class RealTrajectoryLayer(fieldview.Layer):

    def __init__(self, parent = None):
        fieldview.Layer.__init__(self, parent)
        self.name = "Real trajectory"
        self.color = "#edd400"
        self.has_first_goto = False
        self.path = QPainterPath()


    def process_log_line(self, log_line, lineno, last_lineno):
        data = log_line[logger.LOG_LINE_DATA]
        if isinstance(data, packets.KeepAlive):
            pose = data.current_pose
            if not self.has_first_goto:
                self.path.moveTo(pose.y * 1000.0, pose.x * 1000.0)
            else:
                self.path.lineTo(pose.y * 1000.0, pose.x * 1000.0)
        elif isinstance(data, packets.Goto) and data.movement != MOVEMENT_ROTATE:
            self.has_first_goto = True

        if lineno == last_lineno:
            path_item = QGraphicsPathItem(self)
            path_item.setPen(QPen(QColor(self.color), 10))
            path_item.setPath(self.path)




class ExpectedTrajectoryLayer(fieldview.Layer):

    def __init__(self, parent = None):
        fieldview.Layer.__init__(self, parent)
        self.name = "Expected trajectory"
        self.color = "#73d216"
        self.path = QPainterPath()


    def process_log_line(self, log_line, lineno, last_lineno):
        data = log_line[logger.LOG_LINE_DATA]
        if isinstance(data, packets.Resettle):
            if data.axis == AXIS_X:
                self.path.moveTo(self.path.currentPosition().x(), data.position * 1000.0)
                print data.position
            else:
                self.path.moveTo(data.position * 1000.0, self.path.currentPosition().y())
        if isinstance(data, packets.Goto) and data.movement != MOVEMENT_ROTATE:
            for pose in data.points:
                x = pose.x * 1000.0
                y = pose.y * 1000.0
                self.path.lineTo(y, x)

        if lineno == last_lineno:
            path_item = QGraphicsPathItem(self)
            path_item.setPen(QPen(QColor(self.color), 10))
            path_item.setPath(self.path)




class LogFieldViewController(fieldview.FieldViewController):

    def __init__(self, ui):
        fieldview.FieldViewController.__init__(self, ui)

        self.expected_trajectory = ExpectedTrajectoryLayer(self.field_scene)
        self.field_scene.add_layer(self.expected_trajectory)
        self.real_trajectory = RealTrajectoryLayer(self.field_scene)
        self.field_scene.add_layer(self.real_trajectory)

        self.update_layers_list()


    def process_log_line(self, log_line, lineno, last_lineno):
        self.expected_trajectory.process_log_line(log_line, lineno, last_lineno)
        self.real_trajectory.process_log_line(log_line, lineno, last_lineno)

