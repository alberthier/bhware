#!/usr/bin/env python
# encoding: utf-8


from definitions import *

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtSvg import *

import trajectory
import packets
import tools

import fieldview
import helpers
import dynamics




class GraphicsRobotArmObject(QObject):

    arm_movement_finished = pyqtSignal()
    gripper_movement_finished = pyqtSignal()

    def __init__(self):
        QObject.__init__(self)

        self.item = QGraphicsItemGroup()

        self.arm_item = QGraphicsRectItem(0.0, -50.0, 124.0, 100.0);
        self.arm_item.setPen(QColor("#838383"))
        self.arm_item.setBrush(QColor("#e9eaff"))
        self.item.addToGroup(self.arm_item)
        self.is_arm_retracted = True

        self.gripper_item = QGraphicsRectItem(-40.0, -40.0, 40.0, 80.0);
        self.gripper_item.setPen(QPen(0))
        self.gripper_item.setBrush(QColor("#838383"))
        self.gripper_item.setRotation(180.0)
        self.item.addToGroup(self.gripper_item)
        self.is_gripper_retracted = True

        self.arm_animation = QPropertyAnimation()
        self.arm_animation.setDuration(500.0)
        self.arm_animation.setTargetObject(self)
        self.arm_animation.setPropertyName("arm_position")
        self.arm_animation.finished.connect(self.arm_movement_finished)

        self.gripper_animation = QPropertyAnimation()
        self.gripper_animation.setDuration(500.0)
        self.gripper_animation.setTargetObject(self)
        self.gripper_animation.setPropertyName("gripper_position")
        self.gripper_animation.finished.connect(self.gripper_movement_finished)


    def reset(self):
        self.set_arm_position(-99.0)
        self.set_gripper_position(40.0)
        self.is_arm_retracted = True


    def get_arm_position(self):
        return self.item.x()


    def set_arm_position(self, p):
        self.item.setX(p)


    #declare 'arm_position' to Qt's property system
    arm_position = pyqtProperty('qreal', get_arm_position, set_arm_position)


    def get_gripper_position(self):
        return self.gripper_item.rect().width()


    def set_gripper_position(self, w):
        rect = self.gripper_item.rect()
        rect.setWidth(w)
        self.gripper_item.setRect(rect)


    #declare 'gripper_position' to Qt's property system
    gripper_position = pyqtProperty('qreal', get_gripper_position, set_gripper_position)


    def deploy_arm(self):
        self.arm_animation.setStartValue(-99.0)
        self.arm_animation.setEndValue(-224.0)
        self.arm_animation.start()
        self.is_arm_retracted = False


    def retract_arm(self):
        self.arm_animation.setStartValue(-224.0)
        self.arm_animation.setEndValue(-99.0)
        self.arm_animation.start()
        self.is_arm_retracted = True


    def deploy_gripper(self):
        self.gripper_animation.setStartValue(40.0)
        self.gripper_animation.setEndValue(70.0)
        self.gripper_animation.start()
        self.is_gripper_retracted = False


    def retract_gripper(self):
        self.gripper_animation.setStartValue(70.0)
        self.gripper_animation.setEndValue(40.0)
        self.gripper_animation.start()
        self.is_gripper_retracted = True




class GraphicsRobotGripperObject(QObject):

    movement_finished = pyqtSignal()

    def __init__(self, is_right):
        QObject.__init__(self)
        path = QPainterPath()

        if is_right:
            path.arcTo(-61.0, -125.0, 122.0, 125.0, -90.0, 160.0)
            self.item = QGraphicsPathItem(path)
            self.item.setPos(150.0, 125.0)
            self.start_angle = -40
            self.end_angle = 180
        else:
            path.arcTo(-61.0, 125.0, 122.0, -125.0, -90.0, 160.0)
            self.item = QGraphicsPathItem(path)
            self.item.setPos(150.0, -125.0)
            self.start_angle = 40
            self.end_angle = -180
        self.item.setRotation(self.start_angle)

        pen = QPen(QColor("#838383"), 15.0)
        pen.setCapStyle(Qt.RoundCap)
        self.item.setPen(pen)

        self.animation = QPropertyAnimation()
        self.animation.setDuration(1000.0)
        self.animation.setTargetObject(self)
        self.animation.setPropertyName("position")
        self.animation.finished.connect(self.movement_finished)


    def get_position(self):
        return self.item.rotation()


    def set_position(self, p):
        self.item.setRotation(p)


    #declare 'position' to Qt's property system
    position = pyqtProperty('qreal', get_position, set_position)


    def deploy(self):
        self.animation.setStartValue(self.start_angle)
        self.animation.setEndValue(self.end_angle)
        self.animation.start()


    def retract(self):
        self.animation.setStartValue(self.end_angle)
        self.animation.setEndValue(self.start_angle)
        self.animation.start()




class GraphicsRobotSweeperObject(QObject):

    movement_finished = pyqtSignal()

    def __init__(self):
        QObject.__init__(self)

        path = QPainterPath()
        path.arcTo(-67.5, 0.0, 135.0, 180.0, 90.0, 110.0)
        self.left_item = QGraphicsPathItem(path)
        self.left_item.setPos(-70.0, -125.0)

        path = QPainterPath()
        path.arcTo(-67.5, 0.0, 135.0, -180.0, 90.0, 110.0)
        self.right_item = QGraphicsPathItem(path)
        self.right_item.setPos(-70.0, 125.0)

        self.start_angle = 40
        self.end_angle = -140
        self.right_item.setRotation(self.start_angle)
        self.left_item.setRotation(-self.start_angle)

        pen = QPen(QColor("#838383"), 15.0)
        pen.setCapStyle(Qt.RoundCap)
        self.right_item.setPen(pen)
        self.left_item.setPen(pen)

        self.is_retracted = True

        self.animation = QPropertyAnimation()
        self.animation.setDuration(1000.0)
        self.animation.setTargetObject(self)
        self.animation.setPropertyName("position")
        self.animation.finished.connect(self.movement_finished)


    def get_position(self):
        return self.right_item.rotation()


    def set_position(self, p):
        self.right_item.setRotation(p)
        self.left_item.setRotation(-p)


    #declare 'position' to Qt's property system
    position = pyqtProperty('qreal', get_position, set_position)


    def deploy(self):
        self.animation.setStartValue(self.start_angle)
        self.animation.setEndValue(self.end_angle)
        self.animation.start()
        self.is_retracted = False


    def retract(self):
        self.animation.setStartValue(self.end_angle)
        self.animation.setEndValue(self.start_angle)
        self.animation.start()
        self.is_retracted = True




class GraphicsRobotObject(QObject):

    movement_finished = pyqtSignal(int)

    def __init__(self, layer):
        QObject.__init__(self)

        self.move_animation = QParallelAnimationGroup()
        self.move_animation.finished.connect(self.animation_finished)
        self.layer = layer

        self.item = QGraphicsItemGroup(layer)

        self.arm = GraphicsRobotArmObject()
        self.item.addToGroup(self.arm.item)

        self.right_gripper = GraphicsRobotGripperObject(True)
        self.item.addToGroup(self.right_gripper.item)
        self.left_gripper = GraphicsRobotGripperObject(False)
        self.item.addToGroup(self.left_gripper.item)

        self.sweeper = GraphicsRobotSweeperObject()
        self.item.addToGroup(self.sweeper.right_item)
        self.item.addToGroup(self.sweeper.left_item)

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

        self.fabric = QGraphicsRectItem(-60.0, -100.0, 40.0, 200.0)
        self.fabric.setPen(QPen(0))
        self.fabric.setBrush(QColor(layer.color))
        self.item.addToGroup(self.fabric)

        self.carried_treasure = []


    def reset(self):
        for elt in self.carried_treasure:
            self.item.removeFromGroup(elt)
        self.carried_treasure = []
        self.fabric.setVisible(False)
        self.item.setVisible(False)
        self.item.setPos(0.0, 0.0)
        self.set_rotation(0.0)
        self.arm.reset()


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


    def __init__(self, scene, field_view_controller, team):
        fieldview.Layer.__init__(self, scene)
        self.field_view_controller =  field_view_controller
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
        self.robot.arm.arm_movement_finished.connect(self.map_arm_movement_finished)
        self.robot.arm.gripper_movement_finished.connect(self.map_gripper_movement_finished)
        self.robot.left_gripper.movement_finished.connect(self.gripper_movement_finished)
        self.robot.right_gripper.movement_finished.connect(self.gripper_movement_finished)
        self.robot.sweeper.movement_finished.connect(self.sweeper_movement_finished)
        self.robot_controller = None
        self.dynamics = None
        self.expected_gripper_movements = 0
        self.gripper_packet = None


    def reset(self):
        self.robot.reset()


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


    def on_map_arm_control(self, packet):
        if self.robot.arm.is_arm_retracted and packet.move == MAP_ARM_OPEN:
            self.robot.arm.deploy_arm()
        elif not self.robot.arm.is_arm_retracted and packet.move == MAP_ARM_CLOSE:
            self.robot.arm.retract_arm()
        else:
            self.robot_controller.send_packet(packet)


    def map_arm_movement_finished(self):
        packet = packets.MapArmControl()
        if self.robot.arm.is_arm_retracted:
            packet.move = MAP_ARM_CLOSE
        else:
            packet.move = MAP_ARM_OPEN
        self.robot_controller.send_packet(packet)


    def on_map_gripper_control(self, packet):
        if self.robot.arm.is_gripper_retracted and packet.move == MAP_GRIPPER_OPEN:
            self.robot.arm.deploy_gripper()
        elif not self.robot.arm.is_gripper_retracted and packet.move == MAP_GRIPPER_CLOSE:
            self.robot.arm.retract_gripper()
        else:
            self.robot_controller.send_packet(packet)


    def map_gripper_movement_finished(self):
        packet = packets.MapGripperControl()
        if self.robot.arm.is_gripper_retracted:
            packet.move = MAP_GRIPPER_CLOSE
            self.field_view_controller.game_elements_layer.remove_fabric(self.team)
        else:
            packet.move = MAP_GRIPPER_OPEN
        self.robot_controller.send_packet(packet)


    def on_fabric_store_control(self, packet):
        if packet.move == FABRIC_STORE_HIGH:
            self.robot.fabric.show()
        self.robot_controller.send_packet(packet)


    def on_gripper_control(self, packet):
        if packet.which == GRIPPER_SIDE_LEFT or packet.which == GRIPPER_SIDE_BOTH:
            if packet.move == GRIPPER_CLOSE:
                self.robot.left_gripper.retract()
            else:
                self.robot.left_gripper.deploy()
            self.expected_gripper_movements += 1
        if packet.which == GRIPPER_SIDE_RIGHT or packet.which == GRIPPER_SIDE_BOTH:
            if packet.move == GRIPPER_CLOSE:
                self.robot.right_gripper.retract()
            else:
                self.robot.right_gripper.deploy()
            self.expected_gripper_movements += 1
        self.gripper_packet = packet


    def gripper_movement_finished(self):
        self.expected_gripper_movements -= 1
        if self.expected_gripper_movements == 0:
            if self.gripper_packet.which == GRIPPER_SIDE_BOTH and self.gripper_packet.move == GRIPPER_CLOSE:
                pos = self.robot.item.pos()
                if pos.y() < 1000:
                    if pos.x() < 1500:
                        elements = self.field_view_controller.game_elements_layer.purple_map_tower_treasure
                    else:
                        elements = self.field_view_controller.game_elements_layer.red_map_tower_treasure
                else:
                    if pos.x() < 1500:
                        elements = self.field_view_controller.game_elements_layer.purple_bottle_tower_treasure
                    else:
                        elements = self.field_view_controller.game_elements_layer.red_bottle_tower_treasure
                self.robot.carried_treasure = elements
                for elt in elements:
                    elt.setPos(pos)
                    self.robot.item.addToGroup(elt)
            self.robot_controller.send_packet(self.gripper_packet)
            self.gripper_packet = None


    def on_sweeper_control(self, packet):
        if packet.move == SWEEPER_CLOSE:
            self.robot.sweeper.retract()
        else:
            self.robot.sweeper.deploy()


    def sweeper_movement_finished(self):
        packet = packets.SweeperControl()
        if self.robot.sweeper.is_retracted:
            packet.move = SWEEPER_CLOSE
        else:
            packet.move = SWEEPER_OPEN
        self.robot_controller.send_packet(packet)


    def on_empty_tank_control(self, packet):
        angle = self.robot.item.rotation() / 180.0 * math.pi
        dest_x = self.robot.item.x() + math.cos(angle) * 300
        dest_y = self.robot.item.y() + math.sin(angle) * 300
        for elt in self.robot.carried_treasure:
            elt.setPos(dest_x, dest_y)
            self.robot.item.removeFromGroup(elt)
        self.robot_controller.send_packet(packet)


    def userEvent(self, key, x, y):
        dx = x - self.robot.item.x()
        dy = y - self.robot.item.y()
        angle = (self.robot.item.rotation() / 180.0 * math.pi + math.atan2(dy, dx)) % (2.0 * math.pi)
        angle = math.atan2(-math.sin(angle), math.cos(angle)) % ( 2.0 * math.pi )
        angle = int(angle / (2.0 * math.pi) * 18.0)

        packet = packets.TurretDetect()
        packet.angle = angle

        if self.team == TEAM_PURPLE and key == 'q' or self.team == TEAM_RED and key == 's':
            # Main opponent - short distance
            packet.distance = 0
            packet.robot = OPPONENT_ROBOT_MAIN
            self.robot_controller.send_packet(packet)
        elif self.team == TEAM_PURPLE and key == 'Q' or self.team == TEAM_RED and key == 'S':
            # Main opponent - long distance
            packet.distance = 1
            packet.robot = OPPONENT_ROBOT_MAIN
            self.robot_controller.send_packet(packet)
        elif self.team == TEAM_PURPLE and key == 'w' or self.team == TEAM_RED and key == 'x':
            # Secondary opponent - short distance
            packet.distance = 0
            packet.robot = OPPONENT_ROBOT_SECONDARY
            self.robot_controller.send_packet(packet)
        elif self.team == TEAM_PURPLE and key == 'W' or self.team == TEAM_RED and key == 'X':
            # Secondary opponent - long distance
            packet.distance = 1
            packet.robot = OPPONENT_ROBOT_SECONDARY
            self.robot_controller.send_packet(packet)


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



class RoutingLayer(fieldview.Layer):

    def __init__(self, scene, team):
        fieldview.Layer.__init__(self, scene)
        self.team = team
        if self.team == TEAM_PURPLE:
            self.name = "Purple robot routing"
            self.color = TEAM_COLOR_PURPLE
        else:
            self.name = "Red robot routing"
            self.color = TEAM_COLOR_RED
        self.path_blocks = []
        self.walls = []
        #self.setVisible(False)


    def on_simulator_reset_route_path(self, packet):
        for item in self.path_blocks:
            self.scene().removeItem(item)
        self.path_blocks = []


    def on_simulator_route_path(self, packet):
        self.display_path_blocks(packet, QColor(self.color).lighter(120))


    def on_simulator_simplified_route_path(self, packet):
        self.display_path_blocks(packet, QColor(self.color).darker(150))


    def display_path_blocks(self, packet, color):
        cell_size = MAP_CELL_RESOLUTION * 1000.0
        brush = QBrush(color)
        pen = QPen(QBrush(), 0)
        for (x, y) in packet.points:
            item = QGraphicsRectItem(y * cell_size, x * cell_size, cell_size, cell_size)
            item.setBrush(brush)
            item.setPen(pen)
            self.addToGroup(item)
            self.path_blocks.append(item)


    def on_simulator_route_walls(self, packet):
        cell_size = MAP_CELL_RESOLUTION * 1000.0
        brushColor = QColor(QColor("#ab471d"))
        brushColor.setAlpha(80)
        brush = QBrush(brushColor)
        pen = QPen(QBrush(), 0)
        for wall in self.walls:
            self.scene().removeItem(wall)
        self.walls = []
        for (x1, y1, x2, y2) in packet.walls:
            item = QGraphicsRectItem(y1 * cell_size, x1 * cell_size, abs(y2 - y1) * cell_size,  abs(x2 - x1) * cell_size)
            item.setBrush(brush)
            item.setPen(pen)
            self.addToGroup(item)
            self.walls.append(item)





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

        self.purple_map_tower_treasure = [Coin(self, 0.915, 1.015, True),
                                          Coin(self, 0.915, 1.185, True),
                                          Coin(self, 0.915, 1.015, True),
                                          Coin(self, 0.915, 1.185, True),
                                          GoldBar(self, 0.910, 1.100,  0.00)]

        self.purple_bottle_tower_treasure = [Coin(self, 1.085, 1.015, True),
                                             Coin(self, 1.085, 1.185, True),
                                             Coin(self, 1.085, 1.015, True),
                                             Coin(self, 1.085, 1.185, True),
                                             GoldBar(self, 1.090, 1.100,  0.00)]

        self.red_map_tower_treasure = [Coin(self, 0.915, FIELD_Y_SIZE - 1.015, True),
                                       Coin(self, 0.915, FIELD_Y_SIZE - 1.185, True),
                                       Coin(self, 0.915, FIELD_Y_SIZE - 1.015, True),
                                       Coin(self, 0.915, FIELD_Y_SIZE - 1.185, True),
                                       GoldBar(self, 0.910, FIELD_Y_SIZE - 1.100,  0.00)]

        self.red_bottle_tower_treasure = [Coin(self, 1.085, FIELD_Y_SIZE - 1.015, True),
                                          Coin(self, 1.085, FIELD_Y_SIZE - 1.185, True),
                                          Coin(self, 1.085, FIELD_Y_SIZE - 1.015, True),
                                          Coin(self, 1.085, FIELD_Y_SIZE - 1.185, True),
                                          GoldBar(self, 1.090, FIELD_Y_SIZE - 1.100,  0.00)]

        self.elements.extend(self.purple_map_tower_treasure)
        self.elements.extend(self.purple_bottle_tower_treasure)
        self.elements.extend(self.red_map_tower_treasure)
        self.elements.extend(self.red_bottle_tower_treasure)

        for piece in self.elements:
            self.addToGroup(piece)

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
                         # Near hold
                         (1.700, 0.450),
                         # Bottom center
                         (1.700, 1.415)]

        for (x, y) in pieces_coords:
            coin = Coin(self, x, y, True)
            self.addToGroup(coin)
            self.elements.append(coin)
            coin = Coin(self, x, FIELD_Y_SIZE - y, True)
            self.addToGroup(coin)
            self.elements.append(coin)

        for (x, y) in [(1.615, 1.500), (1.785, 1.500)]:
            coin = Coin(self, x, y, True)
            self.addToGroup(coin)
            self.elements.append(coin)

        gold_bars_coords = [(1.353, 1.500,  0.00),
                            (0.860, 0.412, 92.86),
                            (0.860, 2.588, 87.13)]
        for (x, y, angle) in gold_bars_coords:
            bar = GoldBar(self, x, y, angle)
            self.addToGroup(bar)
            self.elements.append(bar)

        self.purple_fabric = Fabric(TEAM_PURPLE)
        self.addToGroup(self.purple_fabric)
        self.elements.append(self.purple_fabric)
        self.red_fabric = Fabric(TEAM_RED)
        self.addToGroup(self.red_fabric)
        self.elements.append(self.red_fabric)

        self.setup()

        scene.changed.connect(self.scene_changed)


    def setup(self):
        self.purple_fabric.show()
        self.red_fabric.show()
        for elt in self.elements:
            elt.setup()


    def scene_changed(self):
        for elt in self.elements:
            if not elt in self.purple_robot.carried_treasure and not elt in self.red_robot.carried_treasure:
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


    def remove_fabric(self, team):
        if team == TEAM_PURPLE:
            self.purple_fabric.hide()
        else:
            self.red_fabric.hide()




class SimulatorFieldViewController(fieldview.FieldViewController):

    def __init__(self, ui):
        fieldview.FieldViewController.__init__(self, ui)

        self.purple_robot_layer = RobotLayer(self.field_scene, self, TEAM_PURPLE)
        self.field_scene.add_layer(self.purple_robot_layer)
        self.purple_robot_trajectrory_layer = RobotTrajectoryLayer(self.field_scene, TEAM_PURPLE)
        self.field_scene.add_layer(self.purple_robot_trajectrory_layer)
        self.purple_robot_routing_layer = RoutingLayer(self.field_scene, TEAM_PURPLE)
        self.field_scene.add_layer(self.purple_robot_routing_layer)

        self.red_robot_layer = RobotLayer(self.field_scene, self, TEAM_RED)
        self.field_scene.add_layer(self.red_robot_layer)
        self.red_robot_trajectrory_layer = RobotTrajectoryLayer(self.field_scene, TEAM_RED)
        self.field_scene.add_layer(self.red_robot_trajectrory_layer)
        self.red_robot_routing_layer = RoutingLayer(self.field_scene, TEAM_RED)
        self.field_scene.add_layer(self.red_robot_routing_layer)

        self.game_elements_layer = GameElementsLayer(self.purple_robot_layer.robot,
                                                     self.red_robot_layer.robot,
                                                     self.field_scene)
        self.field_scene.add_layer(self.game_elements_layer)

        self.field_view.userEventListeners.append(self.purple_robot_layer)
        self.field_view.userEventListeners.append(self.red_robot_layer)

        self.update_layers_list()
