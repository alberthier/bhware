#!/usr/bin/env python
# encoding: utf-8


from definitions import *

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtSvg import *

import trajectory

import fieldview




class GraphicsRobotObject(QObject):

    movement_finished = pyqtSignal()

    def __init__(self, team, parent = None):
        QObject.__init__(self, parent)

        self.team = team

        self.move_animation = QParallelAnimationGroup()
        self.move_animation.finished.connect(self.movement_finished)

        self.observers = []

        self.item = QGraphicsItemGroup()
        self.robot_item = QGraphicsSvgItem(os.path.join(os.path.dirname(__file__), "robot.svg"))
        self.robot_item.setPos(-31.0, -170.0)
        self.item.addToGroup(self.robot_item)
        team_indicator = QGraphicsEllipseItem(31.0, -25.0, 50.0, 50.0)
        if self.team == TEAM_RED:
            color = QColor("#c90000")
        elif self.team == TEAM_PURPLE:
            color = QColor("#0000c9")
        team_indicator.setBrush(color)
        team_indicator.setPen(QPen(0))
        self.item.addToGroup(team_indicator)
        gyration = QGraphicsEllipseItem(-288.0, -288.0, 576.0, 576.0)
        gyration.setPen(QColor("#edd400"))
        self.item.addToGroup(gyration)
        piece_gyration = QGraphicsEllipseItem(-325.0, -325.0, 650.0, 650.0)
        piece_gyration.setPen(QColor("#edd400"))
        self.item.addToGroup(piece_gyration)
        self.left_sensor = Sensor("left")
        self.item.addToGroup(self.left_sensor)
        self.right_sensor = Sensor("right")
        self.item.addToGroup(self.right_sensor)
        self.elevator_sensor = Sensor("elevator")
        self.item.addToGroup(self.elevator_sensor)
        self.back_sensor = Sensor("back")
        self.item.addToGroup(self.back_sensor)
        self.nippers = None


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

        for o in self.observers : o.on_robot_move(self.item.x(), self.item.y(), ref_x, ref_y)

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


    def open_nippers(self):
        if self.nippers == None:
            self.nippers = QGraphicsEllipseItem(self.item.x() - 100.0, self.item.y() -100.0, 200.0, 200.0)
            self.item.addToGroup(self.nippers)


    def close_nippers(self):
        if self.nippers != None:
            self.item.removeFromGroup(self.nippers)
            self.nippers = None




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


    def reset(self):
        pass


    def set_x(self, x):
        pass


    def set_y(self, y):
        pass


    def set_rotation(self, angle):
        pass


    def robot_rotation(self, angle):
        pass


    def robot_line(self, x, y):
        pass


    def robot_move(self, x, y, angle):
        pass



    def get_pose(self):
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


    def reset(self):
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
