# encoding: utf-8


from PyQt4.QtCore import *
from PyQt4.QtGui import *

import collections

import logger
import logtools
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
        packet_type = log_line[logger.LOG_LINE_PACKET]
        if packet_type is packets.KeepAlive:
            pose = logtools.get_value(log_line[logger.LOG_LINE_CONTENT], "current_pose")
            x = logtools.get_value(pose, "x")
            y = logtools.get_value(pose, "y")
            if not self.has_first_goto:
                self.path.moveTo(y * 1000.0, x * 1000.0)
            else:
                self.path.lineTo(y * 1000.0, x * 1000.0)
        if packet_type is packets.Goto:
            movement = logtools.get_value(log_line[logger.LOG_LINE_CONTENT], "movement")
            if movement != MOVEMENT_ROTATE:
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
        packet_type = log_line[logger.LOG_LINE_PACKET]
        if packet_type is packets.Resettle:
            axis = logtools.get_value(log_line[logger.LOG_LINE_CONTENT], "axis")
            position = logtools.get_value(log_line[logger.LOG_LINE_CONTENT], "position")
            if axis == AXIS_X:
                self.path.moveTo(self.path.currentPosition().x(), position * 1000.0)
            else:
                self.path.moveTo(position * 1000.0, self.path.currentPosition().y())
        if packet_type is packets.Goto:
            movement = logtools.get_value(log_line[logger.LOG_LINE_CONTENT], "movement")
            if movement != MOVEMENT_ROTATE:
                for p in logtools.get_value(log_line[logger.LOG_LINE_CONTENT], "points"):
                    x = logtools.get_value(p, "x")
                    y = logtools.get_value(p, "y")
                    self.path.lineTo(y * 1000.0, x * 1000.0)

        if lineno == last_lineno:
            path_item = QGraphicsPathItem(self)
            path_item.setPen(QPen(QColor(self.color), 10))
            path_item.setPath(self.path)




class LogFieldViewController(fieldview.FieldViewController):

    def __init__(self, ui):
        fieldview.FieldViewController.__init__(self, ui)
        self.add_ghost_layer()

        self.expected_trajectory = ExpectedTrajectoryLayer(self.field_scene)
        self.field_scene.add_layer(self.expected_trajectory)
        self.real_trajectory = RealTrajectoryLayer(self.field_scene)
        self.field_scene.add_layer(self.real_trajectory)

        self.update_layers_list()


    def process_log_line(self, log_line, lineno, last_lineno):
        self.expected_trajectory.process_log_line(log_line, lineno, last_lineno)
        self.real_trajectory.process_log_line(log_line, lineno, last_lineno)


    def log_loaded(self):
        pass

