# encoding: utf-8


from definitions import *

import math
import time
import random

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtSvg import *

import position
import packets
import tools
import logger

import fieldview
import helpers




class GraphicsRobotArmObject(QObject):

    def __init__(self, x, size0, size1, size2, robot_object):
        QObject.__init__(self)
        self.robot_object = robot_object

        self.x = x
        self.size0 = size0
        self.size1 = size1
        self.size2 = size2
        self.is_down = False
        self.item = QGraphicsLineItem(robot_object.item)
        self.item.setPen(QPen(QColor("#111111"), 15))

        self.arm_animation = QPropertyAnimation()
        self.arm_animation.setDuration(100.0)
        self.arm_animation.setTargetObject(self)
        self.arm_animation.setPropertyName("position")
        self.arm_animation.finished.connect(self.movement_finished)
        self.current_packet = None


    def setup(self):
        self.set_position(self.size0)


    def get_position(self):
        return self.item.line().y2()


    def set_position(self, p):
        self.item.setLine(self.x, 0.0, self.x, p)


    #declare 'arm_position' to Qt's property system
    position = pyqtProperty('qreal', get_position, set_position)


    def process(self, packet):
        self.current_packet = packet
        size = [self.size0, self.size1, self.size2][packet.position]
        self.is_down = packet.position == 2
        self.arm_animation.setStartValue(self.get_position())
        self.arm_animation.setEndValue(size)
        self.arm_animation.start()


    def movement_finished(self):
        self.robot_object.layer.robot_controller.send_packet(self.current_packet)




class FireDetector(QGraphicsLineItem):

    def __init__(self, x, y, length):
        QGraphicsLineItem.__init__(self, x, y, x, y + length)
        self.setPen(QPen(QBrush(), 1.0, Qt.NoPen))



class GraphicsRobotObject(QObject):

    def __init__(self, layer):
        QObject.__init__(self)

        self.move_animation = QParallelAnimationGroup()
        self.move_animation.stateChanged.connect(self.animation_state_changed)
        self.layer = layer

        self.item = None
        self.robot_item = None
        self.structure = None

        self.carried_elements = []
        self.current_goto_packet = None
        self.current_point_index = 0
        self.stop_requested = False


    def setup(self):
        if self.item is not None:
            for elt in self.carried_elements:
                self.item.removeFromGroup(elt)
            self.carried_elements = []

            self.layer.field_view_controller.field_scene.removeItem(self.item)
            del self.item

        self.item = QGraphicsItemGroup(self.layer)

        arm_idle = 110 if self.layer.robot_controller.is_main else 0

        self.arms = []
        self.left_upper_arm = GraphicsRobotArmObject(-57, -arm_idle, -250, -334, self)
        self.arms.append(self.left_upper_arm)
        self.left_lower_arm = GraphicsRobotArmObject(-47, -arm_idle, -170, -224, self)
        self.arms.append(self.left_lower_arm)
        self.right_upper_arm = GraphicsRobotArmObject(-57, arm_idle, 250, 334, self)
        self.arms.append(self.right_upper_arm)
        self.right_lower_arm = GraphicsRobotArmObject(-47, arm_idle, 170, 224, self)
        self.arms.append(self.right_lower_arm)
        self.gift_arm = GraphicsRobotArmObject(-54, 0, -190, 190, self)
        self.arms.append(self.gift_arm)

        if self.layer.robot_controller.is_main:
            (self.structure, self.robot_item, self.gyration_item) = helpers.create_main_robot_base_item(QColor("#838383"), QColor("#e9eaff"), QColor(self.layer.color).darker(150))
        else:
            (self.structure, self.robot_item, self.gyration_item) = helpers.create_secondary_robot_base_item(QColor("#838383"), QColor("#e9eaff"), QColor(self.layer.color).darker(150))
        self.item.addToGroup(self.structure)

        tower = QGraphicsRectItem(-40.0, -40.0, 80.0, 80.0)
        tower.setPen(QPen(0))
        tower.setBrush(QColor("#838383"))
        self.item.addToGroup(tower)

        self.team_indicator = QGraphicsEllipseItem(-25.0, -25.0, 50.0, 50.0)
        self.team_indicator.setBrush(QColor(self.layer.color))
        self.team_indicator.setPen(QPen(0))
        self.item.addToGroup(self.team_indicator)

        detection_radius = TURRET_SHORT_DISTANCE_DETECTION_RANGE * 1000.0
        self.short_detection_circle = QGraphicsEllipseItem(-detection_radius, -detection_radius, 2.0 * detection_radius, 2.0 * detection_radius)
        self.short_detection_circle.setPen(QPen(QColor(self.layer.color), 2, Qt.DashLine))
        self.item.addToGroup(self.short_detection_circle)

        detection_radius = TURRET_LONG_DISTANCE_DETECTION_RANGE * 1000.0
        self.long_detection_circle = QGraphicsEllipseItem(-detection_radius, -detection_radius, 2.0 * detection_radius, 2.0 * detection_radius)
        self.long_detection_circle.setPen(QPen(QColor(self.layer.color), 2, Qt.DashLine))
        self.item.addToGroup(self.long_detection_circle)

        self.item.setVisible(False)
        self.item.setPos(0.0, 0.0)
        self.set_rotation(0.0)

        for arm in self.arms:
            arm.setup()


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
        return position.Pose(x, y, angle)


    def stop_animation(self):
        self.stop_requested = True
        self.move_animation.stop()


    def pause_animation(self):
        self.move_animation.pause()


    def resume_animation(self):
        self.move_animation.resume()


    def animation_state_changed(self, new_state, old_state):
        if new_state == QAbstractAnimation.Stopped :
            self.process()


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


    def create_arc_animation(self, center_x, center_y, radius, angle):
        r = radius * 1000.0
        cx = center_y * 1000.0
        cy = center_x * 1000.0
        ca = tools.angle_between(cx, cy, self.item.pos().x(), self.item.pos().y())
        da = self.convert_angle(angle) - ca
        duration = abs(r * da)
        pos_animation = QPropertyAnimation()
        pos_animation.setTargetObject(self)
        pos_animation.setPropertyName("position")
        pos_animation.setDuration(duration)
        pos_animation.setStartValue(self.item.pos())
        for i in range(10):
            step = float(i + 1) / 10.0
            x = cx + math.cos(ca + step * da) * r
            y = cy + math.sin(ca + step * da) * r
            pos_animation.setKeyValueAt(step, QPointF(x, y))
        return pos_animation


    def robot_rotate(self, angle):
        rotate_animation = self.create_rotation_animation(angle)
        self.move_animation.clear()
        self.move_animation.addAnimation(rotate_animation)
        self.move_animation.start()


    def robot_line(self, x, y):
        pos_animation = self.create_linear_animation(x, y)
        self.move_animation.clear()
        self.move_animation.addAnimation(pos_animation)
        self.move_animation.start()


    def robot_curve(self, x, y, angle, direction):
        if direction < 0:
            angle += math.pi
        rotate_animation = self.create_rotation_animation(angle)
        pos_animation = self.create_linear_animation(x, y)
        rotate_animation.setDuration(pos_animation.duration())
        self.move_animation.clear()
        self.move_animation.addAnimation(rotate_animation)
        self.move_animation.addAnimation(pos_animation)
        self.move_animation.start()


    def robot_arc(self, center_x, center_y, radius, angle, direction):
        pose = self.get_pose()
        ra = tools.angle_between(center_x, center_y, pose.x, pose.y)
        da = angle + math.copysign(math.pi / 2.0, angle - ra)
        if direction < 0:
            da += math.pi
        rotate_animation = self.create_rotation_animation(da)
        pos_animation = self.create_arc_animation(center_x, center_y, radius, angle)
        rotate_animation.setDuration(pos_animation.duration() * 5)
        pos_animation.setDuration(pos_animation.duration() * 5)
        self.move_animation.clear()
        self.move_animation.addAnimation(rotate_animation)
        self.move_animation.addAnimation(pos_animation)
        self.move_animation.start()


    def on_rotate(self, packet):
        self.layer.robot_controller.send_packet(packets.GotoStarted())
        self.stop_requested = False
        self.robot_rotate(packet.angle)


    def on_move_curve(self, packet):
        self.layer.robot_controller.send_packet(packets.GotoStarted())
        p1 = self.get_pose()
        for p2 in packet.points:
            p2.angle = tools.angle_between(p1.x, p1.y, p2.x, p2.y)
            p1 = p2
        if packet.angle is not None:
            p1.angle = packet.angle
        self.current_goto_packet = packet
        self.stop_requested = False
        self.process()


    def on_move_line(self, packet):
        self.layer.robot_controller.send_packet(packets.GotoStarted())
        self.current_goto_packet = packet
        self.stop_requested = False
        self.process()


    def on_move_arc(self, packet):
        self.layer.robot_controller.send_packet(packets.GotoStarted())
        self.current_goto_packet = packet
        self.stop_requested = False
        self.process()


    def process(self):
        if self.current_goto_packet is None or self.current_point_index == len(self.current_goto_packet.points) or self.stop_requested:
            if self.stop_requested:
                reason = REASON_STOP_REQUESTED
            else:
                reason = REASON_DESTINATION_REACHED
            self.current_goto_packet = None
            self.current_point_index = 0
            p = packets.GotoFinished(reason = reason,
                                     current_pose = self.get_pose(),
                                     current_point_index = self.current_point_index)
            self.layer.robot_controller.send_packet(p)
        else:
            if self.current_point_index != 0:
                self.layer.robot_controller.send_packet(packets.WaypointReached())
            p = self.current_goto_packet.points[self.current_point_index]
            self.current_point_index += 1
            if isinstance(self.current_goto_packet, packets.MoveCurve):
                if len(self.current_goto_packet.points) == self.current_point_index:
                    angle = self.current_goto_packet.angle
                    if self.current_goto_packet.direction < 0:
                        angle += math.pi
                else:
                    angle = p.angle
                self.robot_curve(p.x, p.y, angle, self.current_goto_packet.direction)
            elif isinstance(self.current_goto_packet, packets.MoveLine):
                self.robot_line(p.x, p.y)
            elif isinstance(self.current_goto_packet, packets.MoveArc):
                self.robot_arc(self.current_goto_packet.center.x, self.current_goto_packet.center.y, self.current_goto_packet.radius, p, self.current_goto_packet.direction)
            else:
                raise NotImplementedError("Unimplemented move")


    def on_resettle(self, packet):
        if packet.axis == AXIS_X:
            self.item.setY(packet.position * 1000.0)
        elif packet.axis == AXIS_Y:
            self.item.setX(packet.position * 1000.0)
        angle_deg = self.convert_angle(packet.angle) / math.pi * 180.0
        self.set_rotation(angle_deg)


    def on_candle_kicker(self, packet):
        if packet.side == SIDE_LEFT:
            if packet.which == CANDLE_KICKER_LOWER:
                self.left_lower_arm.process(packet)
            else:
                self.left_upper_arm.process(packet)
        else:
            if packet.which == CANDLE_KICKER_LOWER:
                self.right_lower_arm.process(packet)
            else:
                self.right_upper_arm.process(packet)


    def on_gift_opener(self, packet):
        self.gift_arm.process(packet)


    def hits_gift(self, gift):
        return self.item is not None and gift.collidesWithItem(self.gift_arm.item)


    def hits_upper_candle(self, candle):
        if self.item is not None:
            return self.left_upper_arm.is_down and self.left_upper_arm.item.collidesWithItem(candle) or \
                   self.right_upper_arm.is_down and self.right_upper_arm.item.collidesWithItem(candle)
        return False


    def hits_lower_candle(self, candle):
        if self.item is not None:
            return self.left_lower_arm.is_down and self.left_lower_arm.item.collidesWithItem(candle) or \
                   self.right_lower_arm.is_down and self.right_lower_arm.item.collidesWithItem(candle)
        return False


    def hits_fire(self, fire):
        if fire not in self.carried_elements:
            if self.structure is not None and self.structure.collidesWithItem(fire):
                fire.lay()


    def grab_glass(self, side):
        game_elements_layer = self.layer.field_view_controller.game_elements_layer
        for fire in game_elements_layer.fires:
            if fire not in self.carried_elements:
                if self.structure is not None and self.structure.collidesWithItem(fire):
                    fire.setParentItem(self.item)
                    self.item.addToGroup(fire)
                    self.carried_elements.append(fire)
                    fire.setPos(detector.line().x1() - 38.5, detector.line().y1() + 50 - 38.5)
                    break




class RobotLayer(fieldview.Layer):

    def __init__(self, field_view_controller, robot_controller):
        fieldview.Layer.__init__(self,
                                 field_view_controller,
                                 robot_controller.team_name + " robot",
                                 robot_controller.team_color)
        self.robot_controller = robot_controller
        self.robot = GraphicsRobotObject(self)


    def setup(self):
        self.update_title(self.robot_controller.team_name + " robot", self.robot_controller.team_color)
        self.robot.setup()
        self.robot.item.setVisible(True)


    def get_pose(self):
        return self.robot.get_pose()


    def on_packet(self, packet):
        packet.dispatch(self.robot)


    def key_press_event(self, pos, event):
        if self.robot.item:
            dx = pos.x() - self.robot.item.x()
            dy = pos.y() - self.robot.item.y()
            angle = (self.robot.item.rotation() / 180.0 * math.pi) - math.atan2(dy, dx)
            angle %= 2.0 * math.pi
            angle = int(angle / (2.0 * math.pi) * 18.0)

            packet = packets.TurretDetect()
            packet.angle = angle

            key = event.text()
            team = self.robot_controller.team

            if team == TEAM_YELLOW and key == 'q' or team == TEAM_RED and key == 's':
                # Main opponent - short distance
                packet.distance = 0
                packet.robot = OPPONENT_ROBOT_MAIN
                self.robot_controller.send_packet(packet)
            elif team == TEAM_YELLOW and key == 'Q' or team == TEAM_RED and key == 'S':
                # Main opponent - long distance
                packet.distance = 1
                packet.robot = OPPONENT_ROBOT_MAIN
                self.robot_controller.send_packet(packet)
            elif team == TEAM_YELLOW and key == 'w' or team == TEAM_RED and key == 'x':
                # Secondary opponent - short distance
                packet.distance = 0
                packet.robot = OPPONENT_ROBOT_SECONDARY
                self.robot_controller.send_packet(packet)
            elif team == TEAM_YELLOW and key == 'W' or team == TEAM_RED and key == 'X':
                # Secondary opponent - long distance
                packet.distance = 1
                packet.robot = OPPONENT_ROBOT_SECONDARY
                self.robot_controller.send_packet(packet)




class RobotTrajectoryLayer(fieldview.Layer):

    def __init__(self, field_view_controller, robot_controller):
        fieldview.Layer.__init__(self,
                                 field_view_controller,
                                 robot_controller.team_name + " robot trajectory",
                                 robot_controller.team_color)
        self.robot_controller = robot_controller
        self.item = QGraphicsPathItem()
        self.addToGroup(self.item)


    def setup(self):
        self.update_title(self.robot_controller.team_name + " robot trajectory", self.robot_controller.team_color)
        self.item.setPen(QPen(QColor(self.color), 8.0))
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




class GridRoutingLayer(fieldview.Layer):

    def __init__(self, field_view_controller, robot_controller):
        fieldview.Layer.__init__(self,
                                 field_view_controller,
                                 robot_controller.team_name + " robot grid routing",
                                 robot_controller.team_color)
        self.robot_controller = robot_controller
        self.path_blocks = []
        self.zones = []
        #self.setVisible(False)

        self.main_opponent_zone = self.create_opponent_zone(MAIN_OPPONENT_AVOIDANCE_RANGE * 2.0 * 1000.0)
        self.secondary_opponent_zone = self.create_opponent_zone(SECONDARY_OPPONENT_AVOIDANCE_RANGE * 2.0 * 1000.0)


    def setup(self):
        self.update_title(self.robot_controller.team_name + " robot grid routing", self.robot_controller.team_color)
        self.main_opponent_zone.hide()
        self.secondary_opponent_zone.hide()


    def create_opponent_zone(self, distance):
        group = QGraphicsItemGroup()
        item = QGraphicsEllipseItem(-distance / 2.0, -distance / 2.0, distance, distance)
        item.setPen(QPen(QBrush(), 0))
        brush_color = QColor("#fa0000")
        brush_color.setAlpha(127)
        item.setBrush(brush_color)
        group.addToGroup(item)
        line = QGraphicsLineItem(-50, 0, 50, 0)
        line.setPen(QPen(QColor("#fff000")))
        group.addToGroup(line)
        line = QGraphicsLineItem(0, -50, 0, 50)
        line.setPen(QPen(QColor("#fff000")))
        group.addToGroup(line)
        self.addToGroup(group)
        group.hide()
        return group


    def on_simulator_reset_route_path(self, packet):
        for item in self.path_blocks:
            self.scene().removeItem(item)
        self.path_blocks = []


    def on_simulator_route_path(self, packet):
        self.display_path_blocks(packet, QColor(self.color).lighter(120))


    def on_simulator_simplified_route_path(self, packet):
        self.display_path_blocks(packet, QColor(self.color).darker(150))


    def display_path_blocks(self, packet, color):
        cell_size = ROUTING_MAP_RESOLUTION * 1000.0
        brush = QBrush(color)
        pen = QPen(QBrush(), 0)
        for point in packet.points:
            item = QGraphicsRectItem(point.y * cell_size, point.x * cell_size, cell_size, cell_size)
            item.setBrush(brush)
            item.setPen(pen)
            self.addToGroup(item)
            self.path_blocks.append(item)


    def on_simulator_route_reset_zones(self, packet):
        for zone in self.zones:
            self.scene().removeItem(zone)
        self.zones = []


    def on_simulator_route_rects(self, packet):
        self.add_zone(packet, True)


    def on_simulator_route_circles(self, packet):
        self.add_zone(packet, False)


    def add_zone(self, packet, is_rect):
        cell_size = ROUTING_MAP_RESOLUTION * 1000.0
        if packet.is_forbidden_zone:
            brushColor = QColor(QColor("#ab471d"))
        else:
            brushColor = QColor(QColor("#73ab1d"))
        brushColor.setAlpha(50)
        brush = QBrush(brushColor)
        pen = QPen(QBrush(), 0)

        for shape in packet.shapes:
            if is_rect:
                item = QGraphicsRectItem(shape.y1 * cell_size, shape.x1 * cell_size, abs(shape.y2 - shape.y1) * cell_size,  abs(shape.x2 - shape.x1) * cell_size)
            else:
                item = QGraphicsEllipseItem((shape.y - shape.radius) * cell_size, (shape.x - shape.radius) * cell_size, shape.radius * 2.0 * cell_size, shape.radius * 2.0 * cell_size)
            item.setBrush(brush)
            item.setPen(pen)
            self.addToGroup(item)
            self.zones.append(item)


    def on_simulator_opponents_positions(self, packet):
        if packet.present:
            if packet.robot == OPPONENT_ROBOT_MAIN:
                zone = self.main_opponent_zone
            else:
                zone = self.secondary_opponent_zone
            zone.setPos(packet.y * 1000.0, packet.x * 1000.0)
            zone.show()
        else:
            if packet.robot == OPPONENT_ROBOT_MAIN:
                self.main_opponent_zone.hide()
            else:
                self.secondary_opponent_zone.hide()




class GraphRoutingLayer(fieldview.Layer):

    def __init__(self, field_view_controller, robot_controller):
        fieldview.Layer.__init__(self,
                                 field_view_controller,
                                 robot_controller.team_name + " robot graph routing",
                                 robot_controller.team_color)
        self.robot_controller = robot_controller
        self.edges = []
        self.robot = robot_controller.robot_layer.robot


    def setup(self):
        self.update_title(self.robot_controller.team_name + " robot graph routing", self.robot_controller.team_color)


    def on_simulator_clear_graph_map_edges(self, packet):
        for item in self.edges:
            self.scene().removeItem(item)
        self.edges = []


    def on_simulator_graph_map_edges(self, packet):
        segment_items = 5
        for i in range(len(packet.points) // segment_items):
            y1 = packet.points[i * segment_items] * 1000.0
            x1 = packet.points[i * segment_items + 1] * 1000.0
            y2 = packet.points[i * segment_items + 2] * 1000.0
            x2 = packet.points[i * segment_items + 3] * 1000.0
            penality = packet.points[i * segment_items + 4]
            item = QGraphicsLineItem(x1, y1, x2, y2)
            if tools.quasi_equal(penality, 0.0):
                style = Qt.SolidLine
            else:
                style = Qt.DotLine
            item.setPen(QPen(QColor(self.color).lighter(120), 2, style))
            self.edges.append(item)
            self.addToGroup(item)


    def on_simulator_graph_map_route(self, packet):
        prev_x = self.robot.item.x()
        prev_y = self.robot.item.y()
        point_items = 2
        for i in range(len(packet.points) // point_items):
            y = packet.points[i * point_items] * 1000.0
            x = packet.points[i * point_items + 1] * 1000.0
            item = QGraphicsLineItem(prev_x, prev_y, x, y)
            item.setPen(QPen(QColor(self.color), 20.0, Qt.SolidLine, Qt.RoundCap))
            self.edges.append(item)
            self.addToGroup(item)
            prev_x = x
            prev_y = y




class Fire(QGraphicsPathItem):

    def __init__(self, x, y, game_elements_layer, standing, horizontal, color):
        QGraphicsPathItem.__init__(self)
        self.game_elements_layer = game_elements_layer
        self.x = x
        self.y = y
        self.standing = standing
        self.horizontal = horizontal
        self.color = color

        self.standing_path = QPainterPath()
        self.standing_path.moveTo(-70, -15)
        self.standing_path.lineTo(70, -15)
        self.standing_path.lineTo(70, 15)
        self.standing_path.lineTo(-70, 15)
        self.standing_path.closeSubpath()

        self.laying_path = QPainterPath()
        self.laying_path.moveTo(-70, 36)
        self.laying_path.lineTo(0, -74)
        self.laying_path.lineTo(70, 36)
        self.laying_path.closeSubpath()


    def setup(self):
        if self.standing:
            self.stand()
        else:
            self.lay()
        if self.horizontal:
            self.setRotation(0)
        else:
            self.setRotation(90)
        self.setParentItem(self.game_elements_layer)
        self.setPos(self.x, self.y)


    def stand(self):
        self.setPen(QPen(Qt.NoPen))
        self.setPath(self.standing_path)
        self.setBrush(QBrush(QColor("#010204"), Qt.SolidPattern))


    def lay(self):
        pen = QPen(QBrush(Qt.SolidPattern), 1)
        c = QColor(self.color)
        pen.setColor(c.darker())
        self.setPen(pen)
        self.setPath(self.laying_path)
        self.setBrush(QBrush(c, Qt.SolidPattern))





class GameElementsLayer(fieldview.Layer):

    def __init__(self, field_view_controller):
        fieldview.Layer.__init__(self, field_view_controller, "Game elements", "#f1be01")
        self.main_bar = field_view_controller.ui.main_bar
        self.scene().changed.connect(self.scene_changed)

        self.fires = []
        for x, y, horizontal in [(15, 800, False), (900, 600, False), (400, 1100, True), (900, 1600, False), (1300, 1985, True)]:
            self.fires.append(Fire(x, y, self, True, horizontal, TEAM_COLOR_YELLOW))
            self.fires.append(Fire(3000 - x, y, self, True, horizontal, TEAM_COLOR_RED))
        self.fires.append(Fire(900, 1100, self, False, horizontal, TEAM_COLOR_YELLOW))
        self.fires.append(Fire(2100, 1100, self, False, horizontal, TEAM_COLOR_RED))

        self.elements = self.fires

        for piece in self.elements:
            self.addToGroup(piece)

        self.setup()



    def setup(self):
        for elt in self.elements:
            elt.setup()


    def scene_changed(self):
        robot_a = self.field_view_controller.ui.game_controller.robot_a.robot_layer.robot
        robot_b = self.field_view_controller.ui.game_controller.robot_b.robot_layer.robot
        for elt in self.fires:
            if not elt in robot_a.carried_elements and not elt in robot_b.carried_elements:
                robot = None
                if elt.collidesWithItem(robot_a.robot_item):
                    robot = robot_a
                elif elt.collidesWithItem(robot_b.robot_item):
                    robot = robot_b
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
        for fire in self.fires:
            robot_a.hits_fire(fire)
            robot_b.hits_fire(fire)

        if self.main_bar.opponent_detection.isChecked():
            if robot_a.item and robot_b.item :
                distance = tools.distance(robot_a.item.x(), robot_a.item.y(), robot_b.item.x(), robot_b.item.y())
                if distance < TURRET_SHORT_DISTANCE_DETECTION_RANGE * 1000.0:
                    self.send_turret_detect(robot_a, robot_b, 0)
                    self.send_turret_detect(robot_b, robot_a, 0)
                elif distance < TURRET_LONG_DISTANCE_DETECTION_RANGE * 1000.0:
                    self.send_turret_detect(robot_a, robot_b, 1)
                    self.send_turret_detect(robot_b, robot_a, 1)


    def send_turret_detect(self, detecting_robot, detected_robot, distance):
        dx = detected_robot.item.x() - detecting_robot.item.x()
        dy = detected_robot.item.y() - detecting_robot.item.y()
        angle = (detecting_robot.item.rotation() / 180.0 * math.pi) - math.atan2(dy, dx)
        angle %= 2.0 * math.pi
        angle = int(angle / (2.0 * math.pi) * 18.0)

        packet = packets.TurretDetect()
        packet.distance = distance
        packet.robot = OPPONENT_ROBOT_MAIN
        packet.angle = angle

        detecting_robot.layer.robot_controller.send_packet(packet)




class SimulatorFieldViewController(fieldview.FieldViewController):

    def __init__(self, ui):
        fieldview.FieldViewController.__init__(self, ui)

        fieldview.GhostRobotLayer(self)

        self.game_elements_layer = GameElementsLayer(self)


    def setup(self):
        for layer in self.layers:
            layer.setup()
