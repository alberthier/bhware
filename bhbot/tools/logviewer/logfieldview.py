#!/usr/bin/env python
# encoding: utf-8


from PyQt4.QtCore import *
from PyQt4.QtGui import *

import logger
import packets
from definitions import *

import fieldview




class ExpectedTrajectoryLayer(fieldview.Layer):

    def __init__(self, parent = None):
        fieldview.Layer.__init__(self, parent)
        self.name = "Expected trajectory"
        self.has_first_goto = False
        self.path = QPainterPath()


    def process_log_line(self, log_line, lineno, last_lineno):
        if len(log_line) > logger.LOG_DATA_PACKET:
            packet = log_line[logger.LOG_DATA_PACKET]
            if isinstance(packet, packets.KeepAlive):
                pose = packet.current_pose
                if not self.has_first_goto:
                    self.path.moveTo(pose.y * 1000.0, pose.x * 1000.0)
                else:
                    self.path.lineTo(pose.y * 1000.0, pose.x * 1000.0)
                    print pose.x, pose.y
            elif isinstance(packet, packets.Goto) and packet.movement != MOVEMENT_ROTATE:
                self.has_first_goto = True

        if lineno == last_lineno:
            path_item = QGraphicsPathItem(self)
            path_item.setPen(QPen(QColor("#73d216"), 10))
            path_item.setPath(self.path)




class RealTrajectoryLayer(fieldview.Layer):

    def __init__(self, parent = None):
        fieldview.Layer.__init__(self, parent)
        self.name = "Real trajectory"
        self.has_first_goto = False
        self.path = QPainterPath()


    def process_log_line(self, log_line, lineno, last_lineno):
        if len(log_line) > logger.LOG_DATA_PACKET:
            packet = log_line[logger.LOG_DATA_PACKET]
            if isinstance(packet, packets.Goto) and packet.movement != MOVEMENT_ROTATE:
                for pose in packet.points:
                    x = pose.x * 1000.0
                    y = pose.y * 1000.0
                    if not self.has_first_goto:
                        self.has_first_goto = True
                        self.path.moveTo(y, x)
                    else:
                        self.path.lineTo(y, x)

        if lineno == last_lineno:
            path_item = QGraphicsPathItem(self)
            path_item.setPen(QPen(QColor("#edd400"), 10))
            path_item.setPath(self.path)




class LogFieldViewController(QObject):

    def __init__(self, ui):
        QObject.__init__(self)

        self.ui = ui

        self.field_scene = fieldview.FieldScene(self.ui.trajectory_tab)
        self.expected_trajectory = ExpectedTrajectoryLayer(self.field_scene)
        self.field_scene.add_layer(self.expected_trajectory)
        self.real_trajectory = RealTrajectoryLayer(self.field_scene)
        self.field_scene.add_layer(self.real_trajectory)

        field_view = fieldview.FieldView(self.ui.trajectory_tab)
        field_view.setScene(self.field_scene)
        self.ui.trajectory_tab.layout().addWidget(field_view)


    def process_log_line(self, log_line, lineno, last_lineno):
        self.expected_trajectory.process_log_line(log_line, lineno, last_lineno)
        self.real_trajectory.process_log_line(log_line, lineno, last_lineno)

