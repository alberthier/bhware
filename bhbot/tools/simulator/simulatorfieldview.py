#!/usr/bin/env python
# encoding: utf-8


from definitions import *

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtSvg import *

import trajectory

import fieldview
import helpers




class GraphicsRobotObject(QObject):

    movement_finished = pyqtSignal()

    def __init__(self, layer):
        QObject.__init__(self)

        self.move_animation = QParallelAnimationGroup()
        self.move_animation.finished.connect(self.movement_finished)

        self.item = helpers.create_robot_base_item(QColor("#838383"), QColor("#e9eaff"), QColor(layer.color))
        self.item.setParentItem(layer)

        tower = QGraphicsRectItem(0.0, -40.0, 80.0, 80.0)
        tower.setPen(QPen(0))
        tower.setBrush(QColor("#838383"))
        self.item.addToGroup(tower)

        team_indicator = QGraphicsEllipseItem(15.0, -25.0, 50.0, 50.0)
        team_indicator.setBrush(QColor(layer.color))
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


    def create_rotation_animation(self, angle):
        # Map from robot field reference to Qt reference
        ref_angle = self.convert_angle(angle)
        angle_deg = ((ref_angle) / math.pi * 180.0)

        current = self.item.rotation() % 360.0

        if abs(current - angle_deg) > 180.0:
            if current > angle_deg:
                angle_deg += 360.0
            else:
                angle_deg -= 360.0

        # 360 deg/s
        duration = abs(int((angle_deg - current) / 360.0 * 1000.0))
        rotate_animation = QPropertyAnimation()
        rotate_animation.setTargetObject(self)
        rotate_animation.setPropertyName("angle")
        rotate_animation.setDuration(duration)
        rotate_animation.setStartValue(current)
        rotate_animation.setEndValue(angle_deg)
        return rotate_animation


    def robot_rotation(self, angle):
        rotate_animation = self.create_rotation_animation(angle)
        self.move_animation.clear()
        self.move_animation.addAnimation(rotate_animation)
        self.move_animation.start()


    def create_linear_animation(self, x, y):
        # Map from robot field reference to Qt reference
        ref_x = y * 1000.0
        ref_y = x * 1000.0
        d_field_x = ref_x - self.item.pos().x()
        d_field_y = ref_y - self.item.pos().y()

        # 1 m/s
        duration = math.sqrt(math.pow(d_field_x, 2) + math.pow(d_field_y, 2))
        pos_animation = QPropertyAnimation()
        pos_animation.setTargetObject(self)
        pos_animation.setPropertyName("position")
        pos_animation.setDuration(duration)
        pos_animation.setStartValue(self.item.pos())
        pos_animation.setEndValue(QPointF(ref_x, ref_y))
        return pos_animation


    def robot_line(self, x, y):
        pos_animation = self.create_linear_animation(x, y)
        self.move_animation.clear()
        self.move_animation.addAnimation(pos_animation)
        self.move_animation.start()


    def robot_move(self, x, y, angle):
        rotate_animation = self.create_rotation_animation(angle)
        pos_animation = self.create_linear_animation(x, y)
        rotate_animation.setDuration(pos_animation.duration())
        self.move_animation.clear()
        self.move_animation.addAnimation(rotate_animation)
        self.move_animation.addAnimation(pos_animation)
        self.move_animation.start()




class RobotLayer(fieldview.Layer):

    def __init__(self, parent, team):
        fieldview.Layer.__init__(self, parent)
        self.team = team
        if self.team == TEAM_PURPLE:
            self.name = "Purple robot"
            self.color = TEAM_COLOR_PURPLE
        else:
            self.name = "Red robot"
            self.color = TEAM_COLOR_RED
        self.robot = GraphicsRobotObject(self)
        self.robot.item.setVisible(False)


    def reset(self):
        self.robot.item.setVisible(False)
        self.robot.item.setPos(0.0, 0.0)
        self.robot.set_rotation(0.0)


    def setup(self):
        self.robot.item.setVisible(True)


    def set_x(self, x):
        if self.robot:
            self.robot.item.setY(x * 1000.0)


    def set_y(self, y):
        if self.robot:
            self.robot.item.setX(y * 1000.0)


    def set_rotation(self, angle):
        if self.robot:
            angle_deg = self.robot.convert_angle(angle) / math.pi * 180.0
            print self.name, angle, angle_deg
            self.robot.set_rotation(angle_deg)


    def robot_rotation(self, angle):
        if self.robot:
            self.robot.robot_rotation(angle)


    def robot_line(self, x, y):
        if self.robot:
            self.robot.robot_line(x, y)


    def robot_move(self, x, y, angle):
        if self.robot:
            self.robot.robot_move(x, y, angle)


    def get_pose(self):
        if self.robot:
            return self.robot.get_pose()
        return trajectory.Pose(0.0, 0.0, 0.0)




class RobotTrajectoryLayer(fieldview.Layer):

    def __init__(self, parent, team):
        fieldview.Layer.__init__(self, parent)
        self.team = team
        if self.team == TEAM_PURPLE:
            self.name = "Purple robot trajectory"
            self.color = TEAM_COLOR_PURPLE
        else:
            self.name = "Red robot trajectory"
            self.color = TEAM_COLOR_RED


    def setup(self):
        pass




class GameElementsLayer(fieldview.Layer):

    def __init__(self, parent = None):
        fieldview.Layer.__init__(self, parent)
        self.name = "Game elements"


    def setup(self):
        pass




class SimulatorFieldViewController(fieldview.FieldViewController):

    def __init__(self, ui):
        fieldview.FieldViewController.__init__(self, ui)

        self.purple_robot_layer = RobotLayer(self.field_scene, TEAM_PURPLE)
        self.field_scene.add_layer(self.purple_robot_layer)
        self.purple_robot_trajectrory_layer = RobotTrajectoryLayer(self.field_scene, TEAM_PURPLE)
        self.field_scene.add_layer(self.purple_robot_trajectrory_layer)

        self.red_robot_layer = RobotLayer(self.field_scene, TEAM_RED)
        self.field_scene.add_layer(self.red_robot_layer)
        self.red_robot_trajectrory_layer = RobotTrajectoryLayer(self.field_scene, TEAM_RED)
        self.field_scene.add_layer(self.red_robot_trajectrory_layer)

        self.game_elements_layer = GameElementsLayer(self.field_scene)
        self.field_scene.add_layer(self.game_elements_layer)

        self.update_layers_list()
