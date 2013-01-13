# encoding: utf-8


from definitions import *

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtSvg import *

import position
import packets
import tools

import fieldview
import helpers
import dynamics

import math
import time




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
        self.reset()

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

    def reset(self):
        self.item.setRotation(self.start_angle)

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
        self.move_animation.stateChanged.connect(self.animation_state_changed)
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

        (self.structure, self.robot_item, self.gyration_item) = helpers.create_main_robot_base_item(QColor("#838383"), QColor("#e9eaff"), QColor(layer.color).darker(150))
        self.item.addToGroup(self.structure)
        self.item.setParentItem(layer)

        tower = QGraphicsRectItem(0.0, -40.0, 80.0, 80.0)
        tower.setPen(QPen(0))
        tower.setBrush(QColor("#838383"))
        self.item.addToGroup(tower)

        self.team_indicator = QGraphicsEllipseItem(15.0, -25.0, 50.0, 50.0)
        self.team_indicator.setBrush(QColor(layer.color))
        self.team_indicator.setPen(QPen(0))
        self.item.addToGroup(self.team_indicator)

        detection_radius = TURRET_SHORT_DISTANCE_DETECTION_RANGE * 1000.0
        self.short_detection_circle = QGraphicsEllipseItem(-detection_radius, -detection_radius, 2.0 * detection_radius, 2.0 * detection_radius)
        self.short_detection_circle.setPen(QPen(QColor(layer.color), 2, Qt.DashLine))
        self.item.addToGroup(self.short_detection_circle)

        detection_radius = TURRET_LONG_DISTANCE_DETECTION_RANGE * 1000.0
        self.long_detection_circle = QGraphicsEllipseItem(-detection_radius, -detection_radius, 2.0 * detection_radius, 2.0 * detection_radius)
        self.long_detection_circle.setPen(QPen(QColor(layer.color), 2, Qt.DashLine))
        self.item.addToGroup(self.long_detection_circle)

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
        self.left_gripper.reset()
        self.right_gripper.reset()


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


    def animate(self, points):
        self.move_animation.clear()

        end_time = points[-1][1]
        if end_time == 0.0:
            self.movement_finished.emit(0)
            print("end_time == 0.0 !?")
            return

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

    def animation_state_changed(self, new_state, old_state):
        if new_state == QAbstractAnimation.Stopped :
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


    def __init__(self, field_view_controller, team):
        fieldview.Layer.__init__(self, field_view_controller)
        self.team = team
        if self.team == TEAM_BLUE:
            self.name = "Blue robot"
            self.color = TEAM_COLOR_BLUE
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
                elements_layer = self.field_view_controller.game_elements_layer
                if pos.y() < 1000:
                    if pos.x() < 1500:
                        elements = elements_layer.blue_map_tower_treasure
                        elements_layer.blue_map_tower_treasure_present = False
                    else:
                        elements = elements_layer.red_map_tower_treasure
                        elements_layer.red_map_tower_treasure_present = False
                else:
                    if pos.x() < 1500:
                        elements = elements_layer.blue_bottle_tower_treasure
                        elements_layer.blue_bottle_tower_treasure_present = False
                    else:
                        elements = elements_layer.red_bottle_tower_treasure
                        elements_layer.red_bottle_tower_treasure_present = False
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


    def on_gold_bar_detection(self, packet):
        pos = self.robot.item.pos()
        elements_layer = self.field_view_controller.game_elements_layer
        if pos.y() < 1000:
            if pos.x() < 1500:
                gold_bar_present = elements_layer.blue_map_tower_treasure_present
            else:
                gold_bar_present = elements_layer.red_map_tower_treasure_present
        else:
            if pos.x() < 1500:
                gold_bar_present = elements_layer.blue_bottle_tower_treasure_present
            else:
                gold_bar_present = elements_layer.red_bottle_tower_treasure_present
        if gold_bar_present:
            packet.status = GOLD_BAR_PRESENT
        else:
            packet.status = GOLD_BAR_MISSING
        self.robot_controller.send_packet(packet)


    def userEvent(self, key, x, y):
        dx = x - self.robot.item.x()
        dy = y - self.robot.item.y()
        angle = (self.robot.item.rotation() / 180.0 * math.pi) - math.atan2(dy, dx)
        angle %= 2.0 * math.pi
        angle = int(angle / (2.0 * math.pi) * 18.0)

        packet = packets.TurretDetect()
        packet.angle = angle

        if self.team == TEAM_BLUE and key == 'q' or self.team == TEAM_RED and key == 's':
            # Main opponent - short distance
            packet.distance = 0
            packet.robot = OPPONENT_ROBOT_MAIN
            self.robot_controller.send_packet(packet)
        elif self.team == TEAM_BLUE and key == 'Q' or self.team == TEAM_RED and key == 'S':
            # Main opponent - long distance
            packet.distance = 1
            packet.robot = OPPONENT_ROBOT_MAIN
            self.robot_controller.send_packet(packet)
        elif self.team == TEAM_BLUE and key == 'w' or self.team == TEAM_RED and key == 'x':
            # Secondary opponent - short distance
            packet.distance = 0
            packet.robot = OPPONENT_ROBOT_SECONDARY
            self.robot_controller.send_packet(packet)
        elif self.team == TEAM_BLUE and key == 'W' or self.team == TEAM_RED and key == 'X':
            # Secondary opponent - long distance
            packet.distance = 1
            packet.robot = OPPONENT_ROBOT_SECONDARY
            self.robot_controller.send_packet(packet)


    def terminate(self):
        self.dynamics.terminate()




class RobotTrajectoryLayer(fieldview.Layer):

    def __init__(self, field_view_controller, team):
        fieldview.Layer.__init__(self, field_view_controller)
        self.team = team
        if self.team == TEAM_BLUE:
            self.name = "Blue robot trajectory"
            self.color = QColor(TEAM_COLOR_BLUE).darker(150).name()
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

    def __init__(self, field_view_controller, team):
        fieldview.Layer.__init__(self, field_view_controller)
        self.team = team
        if self.team == TEAM_BLUE:
            self.name = "Blue robot routing"
            self.color = TEAM_COLOR_BLUE
        else:
            self.name = "Red robot routing"
            self.color = TEAM_COLOR_RED
        self.path_blocks = []
        self.zones = []
        #self.setVisible(False)

        self.main_opponent_zone = self.create_opponent_zone(MAIN_OPPONENT_AVOIDANCE_RANGE * 2.0 * 1000.0)
        self.secondary_opponent_zone = self.create_opponent_zone(SECONDARY_OPPONENT_AVOIDANCE_RANGE * 2.0 * 1000.0)


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
        for (x, y) in packet.points:
            item = QGraphicsRectItem(y * cell_size, x * cell_size, cell_size, cell_size)
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


    def reset(self):
        self.main_opponent_zone.hide()
        self.secondary_opponent_zone.hide()




class RoutingGraphLayer(fieldview.Layer):

    def __init__(self, field_view_controller, team, robot):
        fieldview.Layer.__init__(self, field_view_controller)
        self.team = team
        if self.team == TEAM_BLUE:
            self.name = "Blue robot routing graph"
            self.color = TEAM_COLOR_BLUE
        else:
            self.name = "Red robot routing graph"
            self.color = TEAM_COLOR_RED
        self.edges = []
        self.robot = robot
        #self.setVisible(False)


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
        if team == TEAM_BLUE:
            self.setBrush(QColor(TEAM_COLOR_BLUE))
            self.setRect(1250, -135, 250, 148)
        else:
            self.setBrush(QColor(TEAM_COLOR_RED))
            self.setRect(1500, -135, 250, 148)
        self.setPen(QPen(0))


    def setup(self):
        self.setVisible(True)




class GameElementsLayer(fieldview.Layer):

    def __init__(self, field_view_controller):
        fieldview.Layer.__init__(self, field_view_controller)
        self.name = "Game elements"
        self.color = GOLD_BAR_COLOR
        self.elements = []
        self.blue_robot_layer = field_view_controller.blue_robot_layers.robot_layer
        self.red_robot_layer = field_view_controller.red_robot_layers.robot_layer
        self.main_bar = field_view_controller.ui.main_bar
        self.last_sent_turret_detect = None

        self.blue_map_tower_treasure = [Coin(self, 0.915, 1.015, True),
                                          Coin(self, 0.915, 1.185, True),
                                          Coin(self, 0.915, 1.015, True),
                                          Coin(self, 0.915, 1.185, True),
                                          GoldBar(self, 0.910, 1.100,  0.00)]

        self.blue_bottle_tower_treasure = [Coin(self, 1.085, 1.015, True),
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

        self.blue_map_tower_treasure_present = True
        self.blue_bottle_tower_treasure_present = True
        self.red_map_tower_treasure_present = True
        self.red_bottle_tower_treasure_present = True

        self.elements.extend(self.blue_map_tower_treasure)
        self.elements.extend(self.blue_bottle_tower_treasure)
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

        self.blue_fabric = Fabric(TEAM_BLUE)
        self.addToGroup(self.blue_fabric)
        self.elements.append(self.blue_fabric)
        self.red_fabric = Fabric(TEAM_RED)
        self.addToGroup(self.red_fabric)
        self.elements.append(self.red_fabric)

        self.setup()

        self.scene().changed.connect(self.scene_changed)


    def setup(self):
        self.blue_fabric.show()
        self.red_fabric.show()
        for elt in self.elements:
            elt.setup()
        self.blue_map_tower_treasure_present = True
        self.blue_bottle_tower_treasure_present = True
        self.red_map_tower_treasure_present = True
        self.red_bottle_tower_treasure_present = True


    def scene_changed(self):
        for elt in self.elements:
            if not elt in self.blue_robot_layer.robot.carried_treasure and not elt in self.red_robot_layer.robot.carried_treasure:
                robot = None
                if elt.collidesWithItem(self.blue_robot_layer.robot.robot_item):
                    robot = self.blue_robot_layer.robot
                elif elt.collidesWithItem(self.red_robot_layer.robot.robot_item):
                    robot = self.red_robot_layer.robot
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

        if self.main_bar.opponent_detection.isChecked():
            blue_robot_item = self.blue_robot_layer.robot.item
            red_robot_item = self.red_robot_layer.robot.item
            distance = tools.distance(blue_robot_item.x(), blue_robot_item.y(), red_robot_item.x(), red_robot_item.y())
            if distance < TURRET_SHORT_DISTANCE_DETECTION_RANGE * 1000.0:
                self.send_turret_detect(self.blue_robot_layer, self.red_robot_layer, 0)
                self.send_turret_detect(self.red_robot_layer, self.blue_robot_layer, 0)
            elif distance < TURRET_LONG_DISTANCE_DETECTION_RANGE * 1000.0:
                self.send_turret_detect(self.blue_robot_layer, self.red_robot_layer, 1)
                self.send_turret_detect(self.red_robot_layer, self.blue_robot_layer, 1)


    def send_turret_detect(self, detecting_robot_layer, detected_robot_layer, distance):
#        if self.last_sent_turret_detect and self.last_sent_turret_detect + 0.001 > time.time() :
##            print("Rate limiting")
#            return
#        self.last_sent_turret_detect = time.time()
        dx = detected_robot_layer.robot.item.x() - detecting_robot_layer.robot.item.x()
        dy = detected_robot_layer.robot.item.y() - detecting_robot_layer.robot.item.y()
        angle = (detecting_robot_layer.robot.item.rotation() / 180.0 * math.pi) - math.atan2(dy, dx)
        angle %= 2.0 * math.pi
        angle = int(angle / (2.0 * math.pi) * 18.0)

        packet = packets.TurretDetect()
        packet.distance = distance
        packet.robot = OPPONENT_ROBOT_MAIN
        packet.angle = angle

        detecting_robot_layer.robot_controller.send_packet(packet)


    def remove_fabric(self, team):
        if team == TEAM_BLUE:
            self.blue_fabric.hide()
        else:
            self.red_fabric.hide()




class RobotLayersGroup:

    def __init__(self, field_view_controller, team):

        self.layers = []
        self.robot_layer = RobotLayer(field_view_controller, team)
        self.layers.append(self.robot_layer)
        self.robot_trajectory_layer = RobotTrajectoryLayer(field_view_controller, team)
        self.layers.append(self.robot_trajectory_layer)
        self.robot_routing_layer = RoutingLayer(field_view_controller, team)
        self.layers.append(self.robot_routing_layer)
        self.robot_routing_graph_layer = RoutingGraphLayer(field_view_controller, team, self.robot_layer.robot)
        self.layers.append(self.robot_routing_graph_layer)




class SimulatorFieldViewController(fieldview.FieldViewController):

    def __init__(self, ui):
        fieldview.FieldViewController.__init__(self, ui)
        self.ui.main_bar.stop.clicked.connect(self.user_stop)

        fieldview.GhostRobotLayer(self)

        self.blue_robot_layers = RobotLayersGroup(self, TEAM_BLUE)
        self.red_robot_layers = RobotLayersGroup(self, TEAM_RED)

        self.game_elements_layer = GameElementsLayer(self)


    def reset(self):
        for layer in self.layers:
            layer.reset()


    def user_stop(self):
        self.reset()
