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
        self.trajectory_drawer = TrajectoryDrawer(scene,team)


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

            self.view.clear()
            brewery = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))), "brewery", "brewery.py")
            self.process = QProcess()
            self.process.setReadChannelMode(QProcess.MergedChannels)
            self.process.readyRead.connect(self.read_output)
            self.process.start(brewery, [])


    def stop(self):
        if self.process != None:
            self.socket.disconnected.disconnect(self.stop)
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
        self.socket.disconnected.connect(self.stop)
        self.socket.readyRead.connect(self.read_packet)

        self.field_object = GraphicsRobotObject(self.team)
        self.field_object.observers.append(self.trajectory_drawer)
        if self.team == TEAM_RED:
            self.field_object.item.setPos(260.5, 200.0)
        elif self.team == TEAM_BLUE:
            self.field_object.item.setPos(3000.0 - 260.5, 200.0)
            self.field_object.item.setRotation(180.0)
        self.field_object.animation.finished.connect(self.process_goto)
        self.scene.addItem(self.field_object.item)


    def read_output(self):
        while self.process.canReadLine():
            log = str(self.process.readLine()).rstrip()
            self.view.add_log(log)


    def read_packet(self):
        if self.socket != None:
            self.incoming_packet_buffer += self.socket.readAll()
            if self.incoming_packet == None:
                self.incoming_packet = packets.create_packet(self.incoming_packet_buffer)
            if len(self.incoming_packet_buffer) >= self.incoming_packet.MAX_SIZE:
                buf = self.incoming_packet_buffer[:self.incoming_packet.MAX_SIZE]
                self.incoming_packet_buffer = self.incoming_packet_buffer[self.incoming_packet.MAX_SIZE:]
                packet = self.incoming_packet
                self.incoming_packet = None
                packet.deserialize(buf)
                self.on_packet(packet)


    def on_packet(self, packet):
        if isinstance(packet, packets.ControllerReady):
            self.try_device_ready()
        elif isinstance(packet, packets.Goto):
            self.goto_packet = packet
            self.send_packet(packets.GotoStarted())
            self.process_goto()
        elif isinstance(packet, packets.KeepAlive):
            pass
        elif isinstance(packet, packets.PositionControlConfig):
            pass
        elif isinstance(packet, packets.Stop):
            pass
        elif isinstance(packet, packets.Resettle):
            self.send_packet(packet)
        elif isinstance(packet, packets.SimulatorData):
            self.view.handle_led(packet.leds)


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
        self.ready = True
        self.game_controller.try_start()


    def send_keep_alive(self):
        packet = packets.KeepAlive()
        packet.current_pose = self.field_object.get_pose()
        packet.match_started = self.game_controller.started
        packet.match_time = self.game_controller.time
        self.send_packet(packet)


    def send_start_signal(self):
        packet = packets.Start()
        packet.team = self.team
        self.send_packet(packet)


    def process_goto(self):
        if self.goto_packet != None:
            # direction is ignored for the moment
            if len(self.goto_packet.points) != 0:
                point = self.goto_packet.points[0]
                self.goto_packet.points = self.goto_packet.points[1:]
                if self.goto_packet.movement == MOVEMENT_ROTATE:
                    self.field_object.robot_rotation(point.angle)
                elif self.goto_packet.movement == MOVEMENT_MOVE:
                    self.field_object.robot_move(point.x, point.y)
            else:
                self.goto_packet = None
                packet = packets.GotoFinished()
                packet.reason = REASON_DESTINATION_REACHED
                packet.current_pose = self.field_object.get_pose()
                self.send_packet(packet)
