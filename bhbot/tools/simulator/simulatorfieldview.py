#!/usr/bin/env python
# encoding: utf-8


from definitions import *

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtSvg import *

import trajectory
import tools

import fieldview
import helpers
import dynamics




class GraphicsRobotObject(QObject):

    movement_finished = pyqtSignal(int)

    def __init__(self, layer):
        QObject.__init__(self)

        self.move_animation = QParallelAnimationGroup()
        self.move_animation.finished.connect(self.animation_finished)
        self.layer = layer

        self.item = QGraphicsItemGroup(layer)
        (self.structure, self.robot_item, self.gyration_item) = helpers.create_robot_base_item(QColor("#838383"), QColor("#e9eaff"), QColor(layer.color).darker(150))
        self.item.addToGroup(self.structure)
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


    def animate(self, points):
        self.move_animation.clear()

        end_time = points[-1][1]

        rotate_animation = QPropertyAnimation()
        self.move_animation.addAnimation(rotate_animation)
        rotate_animation.setDuration(end_time * 1000.0)
        rotate_animation.setTargetObject(self)
        rotate_animation.setPropertyName("angle")

        pos_animation = QPropertyAnimation()
        self.move_animation.addAnimation(pos_animation)
        pos_animation.setDuration(end_time * 1000.0)
        pos_animation.setTargetObject(self)
        pos_animation.setPropertyName("position")

        current = self.item.rotation() % 360.0
        rotate_animation.setKeyValueAt(0.0, current)
        offset = 0.0
        for (segmentNb, time, pose) in points:
            pos_animation.setKeyValueAt(time / end_time, QPointF(pose.y * 1000.0, pose.x * 1000.0))
            if pose.angle != None:
                ref_angle = self.convert_angle(pose.angle)
                angle_deg = ((ref_angle) / math.pi * 180.0) % 360.0
            else:
                angle_deg = current
            if abs(current - angle_deg) > 180.0:
                if current > angle_deg:
                    offset += 360.0
                else:
                    offset -= 360.0
            current = angle_deg
            angle_deg += offset
            rotate_animation.setKeyValueAt(time / end_time, angle_deg)

        self.move_animation.start()


    def animation_finished(self):
        t = self.move_animation.currentLoopTime() / 1000.0
        self.movement_finished.emit(0)


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


    def __init__(self, scene, team):
        fieldview.Layer.__init__(self, scene)
        self.team = team
        if self.team == TEAM_PURPLE:
            self.name = "Purple robot"
            self.color = TEAM_COLOR_PURPLE
        else:
            self.name = "Red robot"
            self.color = TEAM_COLOR_RED
        self.robot = GraphicsRobotObject(self)
        self.robot.item.setVisible(False)
        self.robot.movement_finished.connect(self.movement_finished)
        self.robot_controller = None
        self.dynamics = None


    def reset(self):
        self.robot.item.setVisible(False)
        self.robot.item.setPos(0.0, 0.0)
        self.robot.set_rotation(0.0)


    def setup(self):
        self.robot.item.setVisible(True)


    def use_advanced_dynamics(self, advanced):
        if advanced:
            self.dynamics = dynamics.PositionControlSimulatorDynamics()
        else:
            self.dynamics = dynamics.BasicDynamics()
        self.dynamics.setup()
        self.dynamics.simulation_finished.connect(self.robot.animate)
        self.dynamics.simulation_finished.connect(self.robot_controller.robot_trajectory_layer.set_points)


    def get_pose(self):
        return self.robot.get_pose()


    def on_goto(self, packet):
        self.dynamics.goto(packet)


    def on_resettle(self, packet):
        if packet.axis == AXIS_X:
            self.robot.item.setY(packet.position * 1000.0)
            self.dynamics.resettle(packet)
        elif packet.axis == AXIS_Y:
            self.robot.item.setX(packet.position * 1000.0)
            self.dynamics.resettle(packet)
        angle_deg = self.robot.convert_angle(packet.angle) / math.pi * 180.0
        self.robot.set_rotation(angle_deg)


    def process_goto(self):
        if self.goto_packet != None:
            if self.goto_packet_point_index != len(self.goto_packet.points):
                point = self.goto_packet.points[self.goto_packet_point_index]
                angle = point.angle
                if angle == None:
                    angle = self.robot.get_pose().angle
                if self.goto_packet.direction == DIRECTION_BACKWARD:
                    angle += math.pi
                if self.goto_packet.movement == MOVEMENT_ROTATE:
                    self.robot.robot_rotation(angle)
                else:
                    self.robot.robot_move(point.x, point.y, angle)
            else:
                self.robot_controller.send_goto_finished(REASON_DESTINATION_REACHED, self.goto_packet_point_index)


    def movement_finished(self, goto_packet_point_index):
        self.robot_controller.send_goto_finished(REASON_DESTINATION_REACHED, goto_packet_point_index)


    def terminate(self):
        self.dynamics.terminate()




class RobotTrajectoryLayer(fieldview.Layer):

    def __init__(self, scene, team):
        fieldview.Layer.__init__(self, scene)
        self.team = team
        if self.team == TEAM_PURPLE:
            self.name = "Purple robot trajectory"
            self.color = QColor(TEAM_COLOR_PURPLE).darker(150).name()
        else:
            self.name = "Red robot trajectory"
            self.color = QColor(TEAM_COLOR_RED).darker(150).name()
        self.item = QGraphicsPathItem()
        self.item.setPen(QPen(QColor(self.color), 8.0))
        self.addToGroup(self.item)
        self.path_blocks = []


    def setup(self):
        self.item.setPath(QPainterPath())


    def on_resettle(self, packet):
        path = self.item.path()
        current = path.currentPosition()
        if packet.axis == AXIS_X:
            path.moveTo(current.x(), packet.position * 1000.0)
        else:
            path.moveTo(packet.position * 1000.0, current.y())
        self.item.setPath(path)


    def set_points(self, points):
        path = self.item.path()
        for point in points:
            pose = point[2]
            path.lineTo(pose.y * 1000.0, pose.x * 1000.0)
        self.item.setPath(path)


    def on_simulator_reset_route_path(self, packet):
        for item in self.path_blocks:
            self.scene().removeItem(item)
        self.path_blocks = []


    def on_simulator_route_path(self, packet):
        cell_size = MAP_CELL_RESOLUTION * 1000.0
        brush = QBrush(QColor(self.color).lighter(180))
        pen = QPen(QBrush(), 0)
        for (x, y) in packet.points:
            item = QGraphicsRectItem(y * cell_size, x * cell_size, cell_size, cell_size)
            item.setBrush(brush)
            item.setPen(pen)
            self.addToGroup(item)
            self.path_blocks.append(item)




class Coin(QGraphicsItemGroup):

    def __init__(self, parent, x, y, is_white):
        QGraphicsItemGroup.__init__(self, parent)
        self.is_white = is_white

        r = COIN_RADIUS * 1000.0
        x = x * 1000.0
        y = y * 1000.0
        path = QPainterPath()
        path.addEllipse(-r, -r, 2.0 * r, 2.0 * r)
        path.addEllipse(-7.5, -7.5, 15.0, 15.0)
        disc = QGraphicsPathItem(path, self)
        if is_white:
            disc.setBrush(QColor(COIN_COLOR_WHITE))
            disc.setPen(QColor(COIN_COLOR_WHITE).darker())
        else:
            disc.setBrush(QColor(COIN_COLOR_BLACK))
            disc.setPen(QColor(COIN_COLOR_BLACK).lighter())
        self.addToGroup(disc)
        self.x = x
        self.y = y


    def setup(self):
        self.setPos(self.y, self.x)




class GoldBar(QGraphicsItemGroup):

    def __init__(self, parent, x, y, angle):
        QGraphicsItemGroup.__init__(self, parent)

        dark_gold = QColor(GOLD_BAR_COLOR).darker()
        w = GOLD_BAR_WIDTH * 1000.0
        l = GOLD_BAR_LENGTH * 1000.0
        x = x * 1000.0
        y = y * 1000.0
        rect = QGraphicsRectItem(-l / 2.0, -w / 2.0, l, w, self)
        rect.setBrush(QColor(GOLD_BAR_COLOR))
        rect.setPen(dark_gold)
        self.addToGroup(rect)
        rect = QGraphicsRectItem(-119.0 / 2.0, -44.0 / 2.0, 119.0, 44.0, self)
        rect.setBrush(QColor(GOLD_BAR_COLOR))
        rect.setPen(dark_gold)
        self.addToGroup(rect)
        line = QGraphicsLineItem(-l / 2.0, -w / 2.0, -119.0 / 2.0, -44.0 / 2.0)
        line.setPen(dark_gold)
        self.addToGroup(line)
        line = QGraphicsLineItem(l / 2.0, -w / 2.0, 119.0 / 2.0, -44.0 / 2.0)
        line.setPen(dark_gold)
        self.addToGroup(line)
        line = QGraphicsLineItem(-l / 2.0, w / 2.0, -119.0 / 2.0, 44.0 / 2.0)
        line.setPen(dark_gold)
        self.addToGroup(line)
        line = QGraphicsLineItem(l / 2.0, w / 2.0, 119.0 / 2.0, 44.0 / 2.0)
        line.setPen(dark_gold)
        self.addToGroup(line)
        self.x = x
        self.y = y
        self.angle = angle


    def setup(self):
        self.setPos(self.y, self.x)
        self.setRotation(self.angle)




class Fabric(QGraphicsRectItem):

    def __init__(self, team, parent = None):
        QGraphicsRectItem.__init__(self, parent)
        if team == TEAM_PURPLE:
            self.setBrush(QColor(TEAM_COLOR_PURPLE))
            self.setRect(1250, -135, 250, 148)
        else:
            self.setBrush(QColor(TEAM_COLOR_RED))
            self.setRect(1500, -135, 250, 148)
        self.setPen(QPen(0))


    def setup(self):
        self.setVisible(True)




class GameElementsLayer(fieldview.Layer):

    def __init__(self, purple_robot, red_robot, scene = None):
        fieldview.Layer.__init__(self, scene)
        self.name = "Game elements"
        self.color = GOLD_BAR_COLOR
        self.elements = []
        self.purple_robot = purple_robot
        self.red_robot = red_robot

        pieces_coords = [# Near start
                         (0.500, 1.000),
                         # Circle
                         (0.830, 1.270),
                         (0.760, 1.100),
                         (0.830, 0.930),
                         (1.000, 0.860),
                         (1.170, 0.930),
                         (1.240, 1.100),
                         (1.170, 1.270),
                         # Totem
                         (0.915, 1.015),
                         (1.085, 1.015),
                         (0.915, 1.185),
                         (1.085, 1.185),
                         (0.915, 1.015),
                         (1.085, 1.015),
                         (0.915, 1.185),
                         (1.085, 1.185),
                         # Near hold
                         (1.700, 0.450),
                         # Bottom center
                         (1.700, 1.415)]

        for (x, y) in pieces_coords:
            coin = Coin(self, x, y, True)
            self.addToGroup(coin)
            self.elements.append(coin)
            coin = Coin(self, x, FIELD_WIDTH - y, True)
            self.addToGroup(coin)
            self.elements.append(coin)

        for (x, y) in [(1.615, 1.500), (1.785, 1.500)]:
            coin = Coin(self, x, y, True)
            self.addToGroup(coin)
            self.elements.append(coin)

        gold_bars_coords = [(1.353, 1.500,  0.00),
                            (0.910, 1.100,  0.00),
                            (1.090, 1.100,  0.00),
                            (0.910, 1.900,  0.00),
                            (1.090, 1.900,  0.00),
                            (0.860, 0.412, 92.86),
                            (0.860, 2.588, 87.13)]
        for (x, y, angle) in gold_bars_coords:
            bar = GoldBar(self, x, y, angle)
            self.addToGroup(bar)
            self.elements.append(bar)

        fabric = Fabric(TEAM_PURPLE)
        self.addToGroup(fabric)
        self.elements.append(fabric)
        fabric = Fabric(TEAM_RED)
        self.addToGroup(fabric)
        self.elements.append(fabric)

        self.setup()

        scene.changed.connect(self.scene_changed)


    def setup(self):
        for elt in self.elements:
            elt.setup()


    def scene_changed(self):
        for elt in self.elements:
            robot = None
            if elt.collidesWithItem(self.purple_robot.robot_item):
                robot = self.purple_robot
            elif elt.collidesWithItem(self.red_robot.robot_item):
                robot = self.red_robot
            if robot != None:
                angle = (robot.item.rotation() / 180.0 * math.pi) % (math.pi * 2.0)

                ex = elt.pos().x() - robot.item.x()
                ey = elt.pos().y() - robot.item.y()
                elt_angle = math.atan2(ey, ex) % (math.pi * 2.0)

                ref = abs(angle - elt_angle)

                if ref < math.pi / 4.0 or ref >= 7.0 * math.pi / 4.0:
                    sign = 1.0
                elif ref >= math.pi / 4.0 and ref < 3.0 * math.pi / 4.0:
                    sign = -1.0
                    angle += math.pi / 2.0
                elif ref >= 3.0 * math.pi / 4.0 and ref < 5.0 * math.pi / 4.0:
                    sign = -1.0
                else:
                    sign = 1.0
                    angle -= math.pi / 2.0

                dist = 20
                dx = sign * math.cos(angle) * dist
                dy = sign * math.sin(angle) * dist
                elt.setPos(elt.pos().x() + dx, elt.pos().y() + dy)




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

        self.game_elements_layer = GameElementsLayer(self.purple_robot_layer.robot,
                                                     self.red_robot_layer.robot,
                                                     self.field_scene)
        self.field_scene.add_layer(self.game_elements_layer)

        self.update_layers_list()
