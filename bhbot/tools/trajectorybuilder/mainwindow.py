#!/usr/bin/env python
# encoding: utf-8


from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4 import uic
import os

import fieldview
import helpers
import math
from definitions import *




SECONDARY_ROBOT_START_X = 0.310
SECONDARY_ROBOT_START_Y = 0.335
GRID_RESOLUTION = 50




class TrajectoryBuilderGhostRobotLayer(fieldview.GhostRobotLayer):

    def __init__(self, parent = None):
        fieldview.GhostRobotLayer.__init__(self, False, parent)
        self.came_from_x = SECONDARY_ROBOT_START_Y * 1000.0
        self.came_from_y = SECONDARY_ROBOT_START_X * 1000.0


    def sceneMouseMoveEvent(self, event):
        pos = event.scenePos()
        grid_x = round(pos.x() / GRID_RESOLUTION) * GRID_RESOLUTION
        grid_y = round(pos.y() / GRID_RESOLUTION) * GRID_RESOLUTION
        dx = grid_x - self.came_from_x
        dy = grid_y - self.came_from_y
        angle = math.atan2(dy, dx)
        if event.modifiers() & Qt.ShiftModifier:
            angle += math.pi
        self.mouse_item.setRotation(angle / math.pi * 180.0)
        self.set_position(grid_x, grid_y)




class PathLayer(fieldview.Layer):

    def __init__(self, ghost, parent = None):
        fieldview.Layer.__init__(self, parent)
        self.ghost = ghost
        self.current_segment = None
        self.add_segment()
        self.color = QColor(TEAM_COLOR_PURPLE).darker(150).name()


    def add_segment(self):
        if self.current_segment != None:
            x = self.current_segment.line().x2()
            y = self.current_segment.line().y2()
        else:
            x = SECONDARY_ROBOT_START_Y * 1000.0
            y = SECONDARY_ROBOT_START_X * 1000.0
        self.current_segment = QGraphicsLineItem(x, y, x, y)
        self.current_segment.setPen(QPen(QColor(self.color), 6.0))
        self.addToGroup(self.current_segment)


    def sceneMouseMoveEvent(self, event):
        current_line = self.current_segment.line()
        self.current_segment.setLine(current_line.x1(), current_line.y1(), self.ghost.pos().x(), self.ghost.pos().y())




class MainWindowController(QObject):

    def __init__(self):
        QObject.__init__(self)

        self.ui = uic.loadUi(os.path.splitext(__file__)[0] + ".ui")
        self.ui.setWindowIcon(QIcon.fromTheme("text-x-generic"))

        self.field_view_controller = fieldview.FieldViewController(self.ui, self)
        self.ghost_layer = TrajectoryBuilderGhostRobotLayer()
        self.field_view_controller.add_ghost_layer(self.ghost_layer)
        self.field_view_controller.field_view.userEventListeners.append(self)

        self.path_layer = PathLayer(self.ghost_layer.mouse_item)
        self.field_view_controller.field_scene.add_layer(self.path_layer)
        self.field_view_controller.field_scene.mouseMoveEventListeners.append(self.path_layer)

        self.ui.show()


    def userEvent(self, button, x, y):
        if button == 'left-button':
            self.ghost_layer.came_from_x = x
            self.ghost_layer.came_from_y = y
            self.path_layer.add_segment()
