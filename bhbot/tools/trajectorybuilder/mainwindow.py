# encoding: utf-8


import os
import math

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4 import uic

import fieldview
import helpers
import tools
from definitions import *




SECONDARY_ROBOT_START_X = 0.310
SECONDARY_ROBOT_START_Y = 0.335
SECONDARY_ROBOT_START_ANGLE = -math.pi / 2.0
GRID_RESOLUTION = 50




class TrajectoryBuilderGhostRobotLayer(fieldview.GhostRobotLayer):

    def __init__(self, parent = None):
        fieldview.GhostRobotLayer.__init__(self, False, parent)
        self.came_from_x = SECONDARY_ROBOT_START_Y * 1000.0
        self.came_from_y = SECONDARY_ROBOT_START_X * 1000.0
        self.forward = True


    def reset_origin(self):
        self.came_from_x = self.mouse_item.pos().x()
        self.came_from_y = self.mouse_item.pos().y()


    def sceneMouseMoveEvent(self, event):
        pos = event.scenePos()
        grid_x = round(pos.x() / GRID_RESOLUTION) * GRID_RESOLUTION
        grid_y = round(pos.y() / GRID_RESOLUTION) * GRID_RESOLUTION
        if abs(grid_x - self.came_from_x) < GRID_RESOLUTION:
            grid_x = self.came_from_x
        if abs(grid_y - self.came_from_y) < GRID_RESOLUTION:
            grid_y = self.came_from_y
        dx = grid_x - self.came_from_x
        dy = grid_y - self.came_from_y
        angle = math.atan2(dy, dx)
        if event.modifiers() & Qt.ShiftModifier:
            angle += math.pi
            self.forward = False
        else:
            self.forward = True
        self.mouse_item.setRotation(angle / math.pi * 180.0)
        self.set_position(grid_x, grid_y)




class PathLayer(fieldview.Layer):

    def __init__(self, ghost, parent = None):
        fieldview.Layer.__init__(self, parent)
        self.ghost = ghost
        self.current_segment = None
        self.add_segment()
        self.color = QColor(TEAM_COLOR_YELLOW).darker(150).name()


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

        self.current_angle = SECONDARY_ROBOT_START_ANGLE

        self.commands = []

        self.ui.show()


    def userEvent(self, button, x, y):
        if button == 'left-button':
            self.ghost_layer.reset_origin()
            self.path_layer.add_segment()

            if self.ghost_layer.forward:
                direction = "MARCHE_AVANT"
            else:
                direction = "MARCHE_ARRIERE"
            angle = self.ghost_layer.convert_angle()
            if not tools.quasi_equal(self.current_angle, angle):
                self.current_angle = angle
                self.commands.append(('rotate_to', angle))
            self.commands.append(('move_to', direction, self.ghost_layer.mouse_item.pos().y() / 1000.0, self.ghost_layer.mouse_item.pos().x() / 1000.0))
        elif button == 'u':
            self.commands.append(('shovel_up',))
        elif button == 'd':
            self.commands.append(('shovel_down',))
        elif button == 'a':
            self.commands.append(('anti_block_on',))
        elif button == 'z':
            self.commands.append(('anti_block_off',))

        self.update_code()


    def update_code(self):
        code = "enum Step {\n"
        for i in range(len(self.commands)):
            code += "    Step{},\n".format(i)
        code += "    StepCount,\n"
        code += "    StepNext,\n"
        code += "    StepEnd\n"
        code += "};\n\n\n"

        code += "static State            statemachine[StepCount]     =\n"
        code += "{\n"
        i = 0
        for command in self.commands:
            if command[0] == 'rotate_to':
                state = "STATE_GOTO_ROTATE({:=0.04f})".format(command[1])
            elif command[0] == 'move_to':
                state = "STATE_GOTO_LINE({}, {:=0.04f}, {:=0.04f})".format(command[1], command[2], command[3])
            elif command[0] == 'shovel_up':
                state = "STATE_SHOVEL_UP"
            elif command[0] == 'shovel_down':
                state = "STATE_SHOVEL_DOWN"
            elif command[0] == 'anti_block_on':
                state = "STATE_ANTI_BLOCK_ON"
            elif command[0] == 'anti_block_off':
                state = "STATE_ANTI_BLOCK_OFF"
            else:
                state = None
                print(("Unknown command {}".format(command)))

            if state != None:
                code += "/* Step{:03} */ {{ {:<50}, StepNext }},\n".format(i, state)
                i += 1
        code += "};\n"

        self.ui.textEdit.setText(code)
