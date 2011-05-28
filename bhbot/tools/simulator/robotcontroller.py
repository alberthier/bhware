#!/usr/bin/env python
# encoding: utf-8

import os
import random

from PyQt4.QtCore import *

import packets
import trajectory
import config
from definitions import *

from robotview import *




class TrajectoryDrawer(object):

    def __init__(self, scene, team) :
        self.scene = scene
        pen_color = Qt.blue if team == TEAM_BLUE else Qt.red
        self.pen = QPen(pen_color, 10, Qt.SolidLine, Qt.SquareCap, Qt.MiterJoin);
        self.items = []


    def on_robot_move(self,ox,oy,x,y):
        if ox and oy :
            l = QGraphicsLineItem(ox,oy,x,y)
            self.items.append(l)
            l.setPen(self.pen)
            self.scene.addItem(l)


    def on_reset(self):
        for i in self.items : self.scene.removeItem(i)
        self.items = []





class SensorController(QObject):

    start_detection = pyqtSignal(QObject)
    end_detection = pyqtSignal(QObject)

    def __init__(self, sensor_item, field_object, sensor_type = None):
        QObject.__init__(self)
        self.enabled = False
        self.currently_detected = []
        self.sensor_item = sensor_item
        self.start_pose = None
        self.end_pose = None
        self.field_object = field_object
        self.sensor_type = sensor_type
        self.sensor_item.scene().changed.connect(self.check)


    def check(self, region):
        if not self.enabled or self.sensor_item.scene() == None:
            return
        only_figures = self.sensor_type == SENSOR_LEFT_TOP or self.sensor_type == SENSOR_RIGHT_TOP
        for piece in self.sensor_item.scene().pieces:
            if not only_figures or piece.piece_type != "P":
                collides = self.sensor_item.collidesWithItem(piece)
                collided = piece in self.currently_detected
                if collides and not collided:
                    self.currently_detected.append(piece)
                    if self.sensor_type == None or len(self.currently_detected) == 1:
                        self.start_pose = self.field_object.get_pose()
                        self.start_detection.emit(self)
                elif not collides and collided:
                    self.currently_detected.remove(piece)
                    if len(self.currently_detected) == 0:
                        self.end_pose = self.field_object.get_pose()
                        self.end_detection.emit(self)




class RobotController(object):

    def __init__(self, team, game_controller, view, scene):
        self.team = team
        self.game_controller = game_controller
        self.view = view
        self.scene = scene

        self.field_object = None
        self.process = None
        self.socket = None
        self.incoming_packet_buffer = ""
        self.incoming_packet = None
        self.ready = False
        self.goto_packet = None
        self.goto_packet_point_index = 0
        self.trajectory_drawer = TrajectoryDrawer(scene, team)

        self.left_sensor = None
        self.right_sensor = None
        self.elevator_sensor = None
        self.back_sensor = None

        self.resettle_count = 0


    def is_process_started(self):
        return self.process != None


    def on_reset(self):
        self.trajectory_drawer.on_reset()


    def is_connected(self):
        return self.socket != None


    def is_ready(self):
        return self.ready


    def setup(self):
        self.on_reset()
        if self.process == None:
            self.field_object = None
            self.incoming_packet_buffer = ""
            self.incoming_packet = None
            self.resettle_count = 0

            self.view.clear()
            brewery = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))), "brewery", "brewery.py")
            self.process = QProcess()
            self.process.setReadChannelMode(QProcess.MergedChannels)
            self.process.readyRead.connect(self.read_output)
            self.process.start(brewery, [])


    def shutdown(self):
        if self.process != None:
            self.socket.disconnected.disconnect(self.shutdown)
            self.process.terminate()
            self.process.waitForFinished()
            self.process = None
            self.socket = None
            self.ready = False


    def remove_field_item(self):
        if self.field_object != None:
            self.scene.removeItem(self.field_object.item)
            self.field_object = None


    def connected(self, socket):
        self.socket = socket
        self.socket.disconnected.connect(self.shutdown)
        self.socket.readyRead.connect(self.read_packet)

        self.field_object = GraphicsRobotObject(self.team)
        self.field_object.observers.append(self.trajectory_drawer)
        self.field_object.movement_finished.connect(self.movement_finished)
        self.scene.addItem(self.field_object.item)

        self.top_left_sensor = SensorController(self.field_object.left_sensor, self.field_object, SENSOR_LEFT_TOP)
        self.top_left_sensor.end_detection.connect(self.on_lateral_detection)
        self.bottom_left_sensor = SensorController(self.field_object.left_sensor, self.field_object, SENSOR_LEFT_BOTTOM)
        self.bottom_left_sensor.end_detection.connect(self.on_lateral_detection)
        self.top_right_sensor = SensorController(self.field_object.right_sensor, self.field_object, SENSOR_RIGHT_TOP)
        self.top_right_sensor.end_detection.connect(self.on_lateral_detection)
        self.bottom_right_sensor = SensorController(self.field_object.right_sensor, self.field_object, SENSOR_RIGHT_BOTTOM)
        self.bottom_right_sensor.end_detection.connect(self.on_lateral_detection)
        self.elevator_sensor = SensorController(self.field_object.elevator_sensor, self.field_object)
        self.elevator_sensor.start_detection.connect(self.on_elevator_detection)
        self.elevator_sensor.enabled = True
        self.back_sensor = SensorController(self.field_object.back_sensor, self.field_object)
        self.back_sensor.start_detection.connect(self.on_back_detection)
        self.back_sensor.enabled = True

        self.front_pieces = []
        self.stored_pieces = []
        self.back_pieces = []



    def read_output(self):
        while self.process.canReadLine():
            log = str(self.process.readLine()).rstrip()
            self.view.add_log(log)


    def read_packet(self):
        if self.socket != None:
            if self.socket.bytesAvailable() > 0 :
                if self.incoming_packet == None:
                    self.incoming_packet = packets.create_packet(self.socket.peek(1))
                if self.socket.bytesAvailable() >= self.incoming_packet.MAX_SIZE:
                    buf = self.socket.read(self.incoming_packet.MAX_SIZE)
                    packet = self.incoming_packet
                    self.incoming_packet = None
                    packet.deserialize(buf)
                    self.on_packet(packet)
                    self.read_packet()

    def on_packet(self, packet):
        if isinstance(packet, packets.ControllerReady):
            self.try_device_ready()
        elif isinstance(packet, packets.Goto):
            self.goto_packet = packet
            self.goto_packet_point_index = 0
            self.send_packet(packets.GotoStarted())
            self.process_goto()
        elif isinstance(packet, packets.KeepAlive):
            pass
        elif isinstance(packet, packets.PositionControlConfig):
            pass
        elif isinstance(packet, packets.Stop):
            self.stop()
        elif isinstance(packet, packets.Resettle):
            if packet.axis == AXIS_ABSCISSA:
                self.field_object.item.setY(packet.position * 1000.0)
            elif packet.axis == AXIS_ORDINATE:
                self.field_object.item.setX(packet.position * 1000.0)
            angle = math.atan2(math.cos(packet.angle), math.sin(packet.angle)) / math.pi * 180.0
            self.field_object.item.setRotation(angle)
            self.send_packet(packet)
            self.resettle_count += 1
            if self.resettle_count == 2:
                self.ready = True
                self.game_controller.try_start()
        elif isinstance(packet, packets.StorePiece1):
            self.send_packet(packet)
        elif isinstance(packet, packets.StorePiece2):
            self.stored_pieces += self.front_pieces
            self.front_pieces = []
            self.send_packet(packet)
        elif isinstance(packet, packets.StorePiece3):
            self.send_packet(packet)
        elif isinstance(packet, packets.ReleasePiece):
            for piece in self.stored_pieces:
                self.field_object.item.removeFromGroup(piece)
            self.stored_pieces = []
            self.send_packet(packet)
        elif isinstance(packet, packets.OpenNippers):
            self.field_object.open_nippers()
            self.send_packet(packet)
        elif isinstance(packet, packets.CloseNippers):
            self.field_object.close_nippers()
            self.send_packet(packet)
        elif isinstance(packet, packets.SimulatorData):
            self.view.handle_led(packet.leds)
        elif isinstance(packet, packets.Deployment):
            self.send_packet(packet)
        elif isinstance(packet, packets.EnableLateralSensors):
            self.top_left_sensor.enabled = True
            self.bottom_left_sensor.enabled = True
            self.top_right_sensor.enabled = True
            self.bottom_right_sensor.enabled = True
            self.send_packet(packet)
        elif isinstance(packet, packets.DisableLateralSensors):
            self.top_left_sensor.enabled = False
            self.bottom_left_sensor.enabled = False
            self.top_right_sensor.enabled = False
            self.bottom_right_sensor.enabled = False
            self.send_packet(packet)
        else :
            print "Unhandled packet type : ",packet.__class__.__name__


    def send_packet(self, packet):
        if self.is_connected():
            self.socket.write(packet.serialize())


    def try_device_ready(self):
        if random.choice([True, False]):
            # OK, ready
            self.send_ready()
        else:
            # Still busy
            packet = packets.DeviceBusy()
            packet.remote_device = REMOTE_DEVICE_SIMULATOR
            self.send_packet(packet)
            QTimer.singleShot(500, self.send_ready)


    def send_ready(self):
        packet = packets.DeviceReady()
        packet.team = self.team
        packet.remote_device = REMOTE_DEVICE_SIMULATOR
        self.send_packet(packet)


    def send_keep_alive(self):
        if self.field_object:
            packet = packets.KeepAlive()
            packet.current_pose = self.field_object.get_pose()
            packet.match_started = self.game_controller.started
            packet.match_time = self.game_controller.time
            self.send_packet(packet)


    def send_start_signal(self):
        packet = packets.Start()
        packet.team = self.team
        self.send_packet(packet)


    def movement_finished(self):
        self.goto_packet_point_index += 1
        self.process_goto()


    def process_goto(self):
        if self.goto_packet != None and self.field_object != None:
            # direction is ignored for the moment
            if self.goto_packet_point_index != len(self.goto_packet.points):
                point = self.goto_packet.points[self.goto_packet_point_index]
                if self.goto_packet.direction == DIRECTION_BACKWARD:
                    point.angle += math.pi
                    for piece in self.front_pieces:
                        self.field_object.item.removeFromGroup(piece)
                    self.front_pieces = []
                elif self.goto_packet.direction == DIRECTION_FORWARD and self.goto_packet.movement != MOVEMENT_ROTATE:
                    for piece in self.back_pieces:
                        self.field_object.item.removeFromGroup(piece)
                    self.back_pieces = []
                if self.goto_packet.movement == MOVEMENT_ROTATE:
                    self.field_object.robot_rotation(point.angle)
                elif self.goto_packet.movement == MOVEMENT_LINE:
                    self.field_object.robot_line(point.x, point.y)
                elif self.goto_packet.movement == MOVEMENT_MOVE:
                    if point.angle != None:
                        self.field_object.robot_move(point.x, point.y, point.angle)
                    else:
                        self.field_object.robot_line(point.x, point.y)
            else:
                self.send_goto_finished(REASON_DESTINATION_REACHED)

    def on_lateral_detection(self, sensor):
        packet = packets.PieceDetected()
        packet.start_pose = sensor.start_pose
        packet.end_pose = sensor.end_pose
        packet.sensor = sensor.sensor_type
        self.send_packet(packet)


    def on_elevator_detection(self, sensor):
        if len(self.stored_pieces) == 0:
            self.field_object.move_animation.stop()
            self.send_goto_finished(REASON_PIECE_FOUND)
        self.front_pieces = self.elevator_sensor.currently_detected
        for piece in self.front_pieces:
            self.field_object.item.addToGroup(piece)


    def on_back_detection(self, sensor):
        self.back_pieces = self.back_sensor.currently_detected
        for piece in self.back_pieces:
            self.field_object.item.addToGroup(piece)


    def pause(self):
        if self.field_object != None and self.field_object.move_animation.state() == QAbstractAnimation.Running:
            self.field_object.move_animation.pause()

    def stop(self):
        if self.field_object != None and self.field_object.move_animation.state() == QAbstractAnimation.Running:
            self.field_object.move_animation.stop()


    def resume(self):
        if self.field_object != None and self.field_object.move_animation.state() == QAbstractAnimation.Paused:
            self.field_object.move_animation.resume()


    def send_goto_finished(self, reason):
        self.goto_packet = None
        packet = packets.GotoFinished()
        packet.reason = reason
        packet.current_pose = self.field_object.get_pose()
        packet.current_point_index = self.goto_packet_point_index
        self.send_packet(packet)

    def send_opponent_detected(self):
        packet = packets.TurretDetect()
        self.send_packet(packet)
